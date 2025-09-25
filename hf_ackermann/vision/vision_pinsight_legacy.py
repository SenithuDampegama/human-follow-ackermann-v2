#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Legacy PiNSIGHT (DepthAI) adapter with NO-CROP preview + correct overlays (1080p wide FOV).

Publishes (offset_x, area, quality) to VisionBus for Layer-5.
This module is kept as a fallback vision source.
"""

from __future__ import annotations
import os, sys, time, threading, subprocess
from typing import Optional, Tuple

AUTO_INSTALL_PKGS = True  # may attempt pip install for depthai/blobconverter

def _import_or_install(modname: str, pipname: Optional[str] = None):
    try:
        return __import__(modname)
    except Exception:
        if not AUTO_INSTALL_PKGS: raise
        pipname = pipname or modname
        print(f"[PiNSightDepthAI] Installing '{pipname}' via pip …")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", pipname])
        return __import__(modname)

# DepthAI (required)
try:
    depthai = _import_or_install("depthai", "depthai")
except Exception as e:
    raise RuntimeError("DepthAI not installed and auto-install failed. Try: pip install depthai") from e

# OpenCV (optional for preview)
try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False

# Reuse VisionBus
from ..control.rc_layer5 import VisionBus


class PiNSightDepthAI(threading.Thread):
    """
    Sensor THE_1080_P (full FOV) -> ISP downscale 1280x720 -> host preview
    ISP -> ImageManip (resize 300x300, keepAspectRatio=True, BGR888p) -> MobileNetSSD
    """
    def __init__(
        self,
        bus: VisionBus,
        blob_path: Optional[str] = None,
        conf_threshold: float = 0.5,
        fps: int = 30,
        video_size: Tuple[int, int] = (1280, 720),
        nn_size: Tuple[int, int] = (300, 300),
        person_label_id: int = 15,
        show_preview: bool = True,
        window_name: str = "PiNSIGHT Preview",
        verbose: bool = True,
    ):
        super().__init__(daemon=True)
        self.bus = bus
        self.conf = float(conf_threshold)
        self.fps = int(fps)
        self.video_size = video_size
        self.nn_size = nn_size
        self.person_label_id = int(person_label_id)
        self.show_preview = bool(show_preview) and _HAVE_CV2
        self.window_name = window_name
        self.verbose = verbose

        self._stop_evt = threading.Event()
        self._blob_path = self._resolve_blob(blob_path)

        # Precompute letterbox mapping from full video -> 300x300
        self._srcW, self._srcH = self.video_size
        self._dstW, self._dstH = self.nn_size
        s = min(self._dstW / self._srcW, self._dstH / self._srcH)
        self._outW = self._srcW * s
        self._outH = self._srcH * s
        self._padX = (self._dstW - self._outW) * 0.5
        self._padY = (self._dstH - self._outH) * 0.5
        # normalized paddings in NN coordinates
        self._padXn = self._padX / self._dstW
        self._padYn = self._padY / self._dstH

    def stop(self): self._stop_evt.set()
    def probe(self): _ = self._build_pipeline()  # raises if blob missing

    # ---------- blob resolution ----------
    def _resolve_blob(self, user_path: Optional[str]) -> Optional[str]:
        if user_path and os.path.isfile(user_path):
            if self.verbose: print(f"[PiNSightDepthAI] Using provided blob: {user_path}")
            return user_path

        here = os.path.dirname(os.path.abspath(__file__))
        for p in [
            os.path.join(here, "models", "mobilenet-ssd_openvino_2021.4_5shave.blob"),
            "/opt/depthai/models/mobilenet-ssd_openvino_2021.4_5shave.blob",
            os.path.expanduser("~/depthai_models/mobilenet-ssd_openvino_2021.4_5shave.blob"),
        ]:
            if os.path.isfile(p):
                if self.verbose: print(f"[PiNSightDepthAI] Found blob: {p}")
                return p

        # auto-download
        try:
            blobconverter = _import_or_install("blobconverter", "blobconverter")
            if self.verbose: print("[PiNSightDepthAI] Downloading MobileNet-SSD blob via blobconverter…")
            path = blobconverter.from_zoo(
                name="mobilenet-ssd",
                shaves=5,
                version="2021.4",
                zoo_type="openvino",
                data_type="FP16",
            )
            if path and os.path.isfile(path):
                if self.verbose: print(f"[PiNSightDepthAI] Blob downloaded: {path}")
                return path
        except Exception as e:
            if self.verbose: print(f"[PiNSightDepthAI] blobconverter failed: {e}")
        return None

    # ---------- pipeline ----------
    def _build_pipeline(self) -> "depthai.Pipeline":
        if not self._blob_path or not os.path.isfile(self._blob_path):
            raise RuntimeError(
                "MobileNet-SSD blob not found. Provide blob_path=, place it in ./models/, "
                "or allow auto-download (requires internet)."
            )

        p = depthai.Pipeline()

        cam = p.create(depthai.node.ColorCamera)

        # Force 1080p full sensor FOV; downscale to 1280x720 via ISP (no crop)
        cam.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setIspScale(2, 3)
        if self.verbose:
            print("[PiNSightDepthAI] Using THE_1080_P + ISP scale 2/3 to 1280x720 (wide FOV).")

        cam.setFps(self.fps)
        cam.setInterleaved(False)
        cam.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)

        # Letterbox to NN input size using ImageManip, ensure BGR planar for the NN
        manip = p.create(depthai.node.ImageManip)
        try:
            bgr_type = depthai.ImgFrame.Type.BGR888p
        except AttributeError:
            bgr_type = depthai.RawImgFrame.Type.BGR888p  # older SDKs

        manip.initialConfig.setResize(self.nn_size[0], self.nn_size[1])
        manip.initialConfig.setKeepAspectRatio(True)   # LETTERBOX (no crop)
        manip.initialConfig.setFrameType(bgr_type)     # NN expects 3×BGR
        manip.setMaxOutputFrameSize(self.nn_size[0] * self.nn_size[1] * 3)

        # Feed ImageManip and preview from ISP (full-FOV, scaled by ISP)
        cam.isp.link(manip.inputImage)

        nn = p.create(depthai.node.MobileNetDetectionNetwork)
        nn.setBlobPath(self._blob_path)
        nn.setConfidenceThreshold(self.conf)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        # NN runs on letterboxed 300x300 BGR planar
        manip.out.link(nn.input)

        # Outputs
        xout_det = p.create(depthai.node.XLinkOut); xout_det.setStreamName("det")
        nn.out.link(xout_det.input)
        xout_rgb = p.create(depthai.node.XLinkOut); xout_rgb.setStreamName("rgb")
        cam.isp.link(xout_rgb.input)

        return p

    # ---------- mapping helpers (letterbox -> full video) ----------
    def _unletterbox(self, x: float, y: float) -> Tuple[float, float]:
        # 16:9 -> square (vertical padding) in our setup
        y = (y - self._padYn) / max(1e-6, (1.0 - 2 * self._padYn))
        # clamp to [0,1]
        x = 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
        y = 0.0 if y < 0.0 else 1.0 if y > 1.0 else y
        return x, y

    # ---------- run ----------
    def run(self):
        try:
            pipeline = self._build_pipeline()
            if self.verbose:
                print("[PiNSightDepthAI] Starting device & detector…")
                if self.show_preview and not _HAVE_CV2:
                    print("[PiNSightDepthAI] OpenCV not available; preview disabled.")

            with depthai.Device(pipeline) as dev:
                q_det = dev.getOutputQueue("det", maxSize=8, blocking=False)
                q_rgb = dev.getOutputQueue("rgb", maxSize=8, blocking=False) if self.show_preview else None

                last_best = None  # (d, conf, area)
                while not self._stop_evt.is_set():
                    # detections
                    pkt = q_det.tryGet()
                    if pkt is not None:
                        best = None
                        best_area = 0.0
                        for d in pkt.detections:
                            if self.person_label_id >= 0 and int(d.label) != self.person_label_id:
                                continue
                            conf = float(d.confidence)
                            if conf < self.conf: continue
                            w = float(d.xmax - d.xmin)
                            h = float(d.ymax - d.ymin)
                            area = max(0.0, min(1.0, w * h))
                            if area > best_area:
                                best_area = area
                                best = (d, conf, area)
                        if best:
                            d, conf, area = best
                            # For control: offset_x in [-1..+1] based on center on the NN frame
                            cx_nn = (float(d.xmin) + float(d.xmax)) * 0.5
                            offset_x = max(-1.0, min(1.0, (cx_nn - 0.5) * 2.0))
                            self.bus.publish(offset_x=offset_x, area=area, quality=conf)
                            last_best = best

                    # preview with overlay
                    if self.show_preview and q_rgb is not None:
                        fb = q_rgb.tryGet()
                        if fb is not None:
                            frame = fb.getCvFrame()  # BGR
                            H, W = frame.shape[:2]
                            if last_best and _HAVE_CV2:
                                d, conf, area = last_best
                                # Unletterbox detection coords back to video space
                                x1n, y1n = self._unletterbox(float(d.xmin), float(d.ymin))
                                x2n, y2n = self._unletterbox(float(d.xmax), float(d.ymax))
                                x1 = int(x1n * W); y1 = int(y1n * H)
                                x2 = int(x2n * W); y2 = int(y2n * H)

                                # Draw bbox + center line
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.line(frame, (W//2, 0), (W//2, H), (255, 255, 255), 1)
                                cx_v = (x1 + x2) * 0.5 / W
                                offx = (cx_v - 0.5) * 2.0
                                cv2.putText(
                                    frame,
                                    f"conf={conf:.2f} area={area:.3f} offx={offx:+.2f}",
                                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2
                                )

                            if _HAVE_CV2:
                                cv2.imshow(self.window_name, frame)
                                key = cv2.waitKey(1) & 0xFF
                                if key == 27 or key == ord('q'):
                                    self._stop_evt.set()
                                    break

                    time.sleep(0.002)

        except Exception as e:
            print(f"[PiNSightDepthAI] ERROR: {e}")
        finally:
            if self.show_preview and _HAVE_CV2:
                try: cv2.destroyWindow(self.window_name)
                except Exception: pass

