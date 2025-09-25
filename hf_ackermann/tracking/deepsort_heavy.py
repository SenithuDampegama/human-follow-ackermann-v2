from __future__ import annotations
from typing import List, Tuple, Optional


class NotAvailable(RuntimeError):
    pass


class HeavyDeepSort:
    """
    Optional wrapper around deep-sort-realtime to match the lightweight IOUTracker API.
    update(detections, image_size) -> List[(track_id, bbox, conf)]
    """
    def __init__(self):
        try:
            from deep_sort_realtime.deepsort_tracker import DeepSort  # type: ignore
        except Exception as e:
            raise NotAvailable(
                "deep-sort-realtime is not installed. Install it or use --use-tracker iou."
            ) from e
        # Reasonable defaults for CPU
        self._ds = DeepSort(max_age=10, n_init=3, nms_max_overlap=1.0, max_cosine_distance=0.2)

    def update(self, detections: List[Tuple[Tuple[int,int,int,int], float]], image_size: Tuple[int,int]) \
            -> List[Tuple[int, Tuple[int,int,int,int], float]]:
        # deep-sort-realtime expects detections as [ [x1,y1,x2,y2,conf], ... ] and returns tracks
        det_list = [[x1,y1,x2,y2,conf] for (x1,y1,x2,y2), conf in detections]
        tracks = self._ds.update_tracks(det_list, frame=None, frame_shape=(image_size[1], image_size[0], 3))
        out = []
        for t in tracks:
            if not t.is_confirmed():
                continue
            tid = int(t.track_id)
            ltrb = t.to_ltrb()  # [l,t,r,b]
            x1, y1, x2, y2 = map(int, ltrb)
            conf = float(getattr(t, 'det_conf', 1.0))
            out.append((tid, (x1,y1,x2,y2), conf))
        return out

