#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OAK-D Pro W on-camera MobileNetSpatialDetectionNetwork + lightweight IOU tracker.

Publishes only the current target to VisionBus, including track_id and z_m (meters) when available.
Optional OpenCV preview with lock/unlock hotkeys.
"""

from __future__ import annotations
import os
import sys
import time
import threading
from typing import Callable, Optional, Tuple, List

# Try import depthai (required)
try:
    import depthai as dai
except Exception as e:
    raise RuntimeError("depthai is required. Install with: pip install depthai") from e

# Try import cv2 (optional)
try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False

import numpy as np

from ..control.rc_layer5 import VisionBus
from ..tracking.deepsort_iou import IOUTracker


class OAKDSpatial(threading.Thread):
    def __init__(
        self,
        bus: VisionBus,
        blob_path: Optional[str] = None,
        conf_threshold: float = 0.5,
        fps: int = 30,
        video_size: Tuple[int, int] = (1280, 720),
        nn_size: Tuple[int, int] = (300, 300),
        person_label_id: int = 15,
        show_preview: bool = False,
        window_name: str = "OAK-D | RGB + Depth",
        use_tracker: str = "iou",
        lock_cb: Optional[Callable[[Optional[int]], None]] = None,
        release_cb: Optional[Callable[[], None]] = None,
        get_locked_id: Optional[Callable[[], Optional[int]]] = None,
        verbose: bool = True,
        depth_range_m: Optional[Tuple[float, float]] = (0.3, 4.0),
        preview_scale: float = 0.6,
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
        # Depth visualization range (near, far) in meters
        if depth_range_m is None:
            self._depth_near_m, self._depth_far_m = 0.3, 4.0
        else:
            self._depth_near_m, self._depth_far_m = float(depth_range_m[0]), float(depth_range_m[1])
        # Preview scale for display-only resizing
        try:
            self.preview_scale = float(preview_scale)
        except Exception:
            self.preview_scale = 0.6

        # tracker
        if use_tracker.lower() == "iou":
            self.tracker = IOUTracker(max_age=8, iou_threshold=0.2, min_hits=3)
        else:
            # Optionally add heavy tracker wrapper later; keep API consistent
            from ..tracking.deepsort_heavy import HeavyDeepSort
            self.tracker = HeavyDeepSort()

        # Lock integration
        self._lock_cb = lock_cb
        self._release_cb = release_cb
        self._get_locked_id = get_locked_id

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
            if self.verbose: print(f"[OAKDSpatial] Using provided blob: {user_path}")
            return user_path
        # Try blobconverter if available
        try:
            import blobconverter  # type: ignore
            if self.verbose: print("[OAKDSpatial] Downloading MobileNet-SSD blob via blobconverter…")
            path = blobconverter.from_zoo(
                name="mobilenet-ssd",
                shaves=5,
                version="2021.4",
                zoo_type="openvino",
                data_type="FP16",
            )
            if path and os.path.isfile(path):
                if self.verbose: print(f"[OAKDSpatial] Blob downloaded: {path}")
                return path
        except Exception as e:
            if self.verbose:
                print(f"[OAKDSpatial] blobconverter unavailable: {e}. You may pass --blob PATH")
        return user_path  # could be None; MobileNetSpatialDetectionNetwork will error in probe()

    # ---------- pipeline ----------
    def _resolve_stereo_preset(self):
        """Resolve StereoDepth.PresetMode enum across DepthAI versions.
        Prefer modern location; fallback to legacy namespace.
        """
        # Try modern path first
        for attr in ("StereoDepth", "node.StereoDepth"):
            try:
                ns = eval(f"dai.{attr}.PresetMode")
                return ns
            except Exception:
                pass
        raise RuntimeError("StereoDepth.PresetMode enum not found in this DepthAI build")

    def _build_pipeline(self) -> "dai.Pipeline":
        if not self._blob_path or not os.path.isfile(self._blob_path):
            raise RuntimeError(
                "MobileNet-SSD blob not found. Provide --blob, place in ./models/, or allow auto-download."
            )

        p = dai.Pipeline()

        cam = p.create(dai.node.ColorCamera)
        # Explicit modern enum paths
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setFps(self.fps)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        # We use preview 300x300 for NN with keepAspect, and ISP 1280x720 for nice preview
        cam.setPreviewSize(self.nn_size[0], self.nn_size[1])
        cam.setPreviewKeepAspectRatio(True)
        cam.setIspScale(2, 3)  # 1920x1080 -> 1280x720

        monoL = p.create(dai.node.MonoCamera)
        monoR = p.create(dai.node.MonoCamera)
        monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo = p.create(dai.node.StereoDepth)
        # Version-agnostic PresetMode resolver and selection
        PresetMode = self._resolve_stereo_preset()
        if hasattr(PresetMode, "HIGH_DENSITY"):
            stereo.setDefaultProfilePreset(PresetMode.HIGH_DENSITY)
        elif hasattr(PresetMode, "HIGH_ACCURACY"):
            stereo.setDefaultProfilePreset(PresetMode.HIGH_ACCURACY)
        else:
            stereo.setDefaultProfilePreset(PresetMode.BEST)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setSubpixel(True)
        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        nn = p.create(dai.node.MobileNetSpatialDetectionNetwork)
        nn.setBlobPath(self._blob_path)
        nn.setConfidenceThreshold(self.conf)
        nn.input.setBlocking(False)
        nn.setBoundingBoxScaleFactor(0.5)
        nn.setDepthLowerThreshold(100)   # mm
        nn.setDepthUpperThreshold(10000) # mm

        cam.preview.link(nn.input)
        stereo.depth.link(nn.inputDepth)

        # Outputs
        xout_det = p.create(dai.node.XLinkOut); xout_det.setStreamName("det")
        nn.out.link(xout_det.input)

        xout_rgb = p.create(dai.node.XLinkOut); xout_rgb.setStreamName("rgb")
        cam.isp.link(xout_rgb.input)

        # Depth output (aligned to RGB) for preview only
        xout_depth = p.create(dai.node.XLinkOut); xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return p

    # ---------- mapping helpers ----------
    def _unletterbox(self, x: float, y: float) -> Tuple[float, float]:
        # Map NN-normalized coords (0..1, including padding) to video-normalized (0..1)
        # Vertical padding expected (16:9 to square)
        y = (y - self._padYn) / max(1e-6, (1.0 - 2 * self._padYn))
        x = 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
        y = 0.0 if y < 0.0 else 1.0 if y > 1.0 else y
        return x, y

    # ---------- run loop ----------
    def run(self):
        try:
            pipeline = self._build_pipeline()
            # Diagnostics: DepthAI version
            try:
                print(f"[OAKDSpatial] DepthAI version: {getattr(dai, '__version__', '?')}")
            except Exception:
                pass
            if self.verbose:
                print("[OAKDSpatial] Starting device & spatial detector…")
                if self.show_preview and not _HAVE_CV2:
                    print("[OAKDSpatial] OpenCV not available; preview disabled.")

            with dai.Device(pipeline) as dev:
                # Diagnostics: device name + MXID
                try:
                    name = None
                    mxid = None
                    try:
                        name = dev.getDeviceName()
                    except Exception:
                        pass
                    try:
                        mxid = dev.getMxId() if hasattr(dev, 'getMxId') else dev.getDeviceInfo().getMxId()
                    except Exception:
                        pass
                    if name or mxid:
                        print(f"[OAKDSpatial] Device: {name or 'Unknown'}  MXID: {mxid or 'Unknown'}")
                except Exception:
                    pass
                q_det = dev.getOutputQueue("det", maxSize=8, blocking=False)
                q_rgb = dev.getOutputQueue("rgb", maxSize=8, blocking=False) if self.show_preview else None
                q_depth = dev.getOutputQueue("depth", maxSize=2, blocking=False) if self.show_preview else None

                last_frame = None
                last_tracks: List[Tuple[int, Tuple[int,int,int,int], float, float]] = []
                last_target: Optional[Tuple[int, Tuple[int,int,int,int], float, float]] = None

                while not self._stop_evt.is_set():
                    # Preview frame
                    if q_rgb is not None:
                        fb = q_rgb.tryGet()
                        if fb is not None:
                            last_frame = fb.getCvFrame()

                    # Detections
                    pkt = q_det.tryGet()
                    tracks: List[Tuple[int, Tuple[int,int,int,int], float, float]] = []
                    # items: (track_id, bbox_xyxy, conf, z_m)
                    if pkt is not None:
                        dets = []  # (bbox_xyxy, conf)
                        zs = []    # z in meters aligned to dets
                        for d in pkt.detections:
                            if self.person_label_id >= 0 and int(d.label) != self.person_label_id:
                                continue
                            conf = float(d.confidence)
                            if conf < self.conf:
                                continue
                            # bbox in NN normalized (including paddings)
                            x1n, y1n = float(d.xmin), float(d.ymin)
                            x2n, y2n = float(d.xmax), float(d.ymax)
                            # Convert to video pixel coords for tracking/overlay
                            x1v, y1v = self._unletterbox(x1n, y1n)
                            x2v, y2v = self._unletterbox(x2n, y2n)
                            W, H = self.video_size
                            x1 = int(max(0, min(W-1, x1v * W)))
                            y1 = int(max(0, min(H-1, y1v * H)))
                            x2 = int(max(0, min(W-1, x2v * W)))
                            y2 = int(max(0, min(H-1, y2v * H)))
                            if x2 <= x1 or y2 <= y1:
                                continue
                            dets.append(((x1, y1, x2, y2), conf))
                            z_m = max(0.0, float(d.spatialCoordinates.z) / 1000.0)
                            zs.append(z_m)

                        # Update tracker
                        if dets:
                            tr = self.tracker.update(dets, image_size=self.video_size)
                            # Build tracks list with z
                            for (tid, bbox, conf), z in zip(tr, zs):
                                tracks.append((tid, bbox, conf, z))

                        # Choose current target
                        target = None
                        if tracks:
                            # If L5 has a locked id and it's present, pick it.
                            locked = self._get_locked_id() if self._get_locked_id else None
                            if locked is not None:
                                for tid, bbox, conf, z in tracks:
                                    if tid == locked:
                                        target = (tid, bbox, conf, z)
                                        break
                            # Else pick the nearest to center (min |offset_x|)
                            if target is None:
                                def offx_of(b):
                                    x1, y1, x2, y2 = b
                                    cx = (x1 + x2) * 0.5 / self.video_size[0]
                                    return abs((cx - 0.5) * 2.0)
                                target = min(tracks, key=lambda t: offx_of(t[1]))

                        # Publish only the target
                        if target is not None:
                            tid, bbox, conf, z_m = target
                            x1, y1, x2, y2 = bbox
                            cx = (x1 + x2) * 0.5 / self.video_size[0]
                            cy = (y1 + y2) * 0.5 / self.video_size[1]
                            offx = max(-1.0, min(1.0, (cx - 0.5) * 2.0))
                            area = ((x2 - x1) * (y2 - y1)) / float(self.video_size[0] * self.video_size[1])
                            self.bus.publish(offset_x=offx, area=area, quality=conf, track_id=tid, z_m=z_m)
                            last_target = target
                        last_tracks = tracks

                    # Draw preview
                    if self.show_preview and _HAVE_CV2 and last_frame is not None:
                        rgb_frame = last_frame.copy()
                        H, W = rgb_frame.shape[:2]
                        # overlay center line (vertical)
                        cv2.line(rgb_frame, (W//2, 0), (W//2, H), (255,255,255), 1)
                        # Draw tracks and target with label and short horizontal center line
                        if last_tracks:
                            locked = self._get_locked_id() if self._get_locked_id else None
                            for tid, bbox, conf, z in last_tracks:
                                x1, y1, x2, y2 = bbox
                                color = (0, 255, 0)
                                thickness = 2
                                if locked is not None and tid == locked:
                                    color = (0, 255, 255)
                                    thickness = 3
                                if last_target is not None and tid == last_target[0]:
                                    color = (0, 128, 255)
                                    thickness = 3
                                cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), color, thickness)
                                cx_px = int((x1 + x2) * 0.5)
                                cy_px = int((y1 + y2) * 0.5)
                                # short horizontal line at bbox center
                                cv2.line(rgb_frame, (max(0, cx_px-12), cy_px), (min(W-1, cx_px+12), cy_px), color, 2)
                                offx = ((cx_px / W) - 0.5) * 2.0
                                label = f"id:{tid} conf:{conf:.2f} z:{z:.2f}m offx:{offx:+.2f}"
                                cv2.putText(rgb_frame, label, (x1, max(0, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                        # Depth visualization (false-color), only if queue exists
                        depth_color = None
                        try:
                            if q_depth is not None:
                                dep_pkt = q_depth.tryGet()
                                if dep_pkt is not None:
                                    depth_mm = dep_pkt.getFrame()  # uint16 in mm
                                    if depth_mm is not None:
                                        depth_m = depth_mm.astype(np.float32) / 1000.0
                                        n = (depth_m - self._depth_near_m) / max(1e-6, (self._depth_far_m - self._depth_near_m))
                                        n = np.clip(n, 0.0, 1.0)
                                        depth_u8 = (n * 255.0).astype(np.uint8)
                                        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
                        except Exception:
                            depth_color = None

                        if depth_color is None:
                            # fallback empty depth view matching size
                            depth_color = np.zeros_like(rgb_frame)
                        else:
                            depth_color = cv2.resize(depth_color, (W, H), interpolation=cv2.INTER_NEAREST)

                        composite = np.hstack([rgb_frame, depth_color])
                        # Display-only downscale for smaller window while preserving fidelity
                        if self.preview_scale != 1.0:
                            composite = cv2.resize(
                                composite, None,
                                fx=self.preview_scale, fy=self.preview_scale,
                                interpolation=cv2.INTER_AREA
                            )
                        cv2.imshow(self.window_name, composite)
                        key = cv2.waitKey(1) & 0xFF
                        if key == 27 or key == ord('q'):
                            self._stop_evt.set()
                            break
                        elif key == ord('r'):
                            if self._release_cb:
                                self._release_cb()
                        elif key == ord('l'):
                            # Lock the nearest track by center if any was seen this packet
                            if last_tracks:
                                def offx_of(b):
                                    x1, y1, x2, y2 = b
                                    cx = (x1 + x2) * 0.5 / self.video_size[0]
                                    return abs((cx - 0.5) * 2.0)
                                tid, bbox, conf, z = min(last_tracks, key=lambda t: offx_of(t[1]))
                                if self._lock_cb:
                                    self._lock_cb(tid)

                    time.sleep(0.002)

        except Exception as e:
            print(f"[OAKDSpatial] ERROR: {e}")
        finally:
            if self.show_preview and _HAVE_CV2:
                try: cv2.destroyWindow(self.window_name)
                except Exception: pass
