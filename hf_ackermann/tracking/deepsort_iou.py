from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Optional


def _iou(boxA: Tuple[int,int,int,int], boxB: Tuple[int,int,int,int]) -> float:
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interW = max(0, xB - xA)
    interH = max(0, yB - yA)
    inter = interW * interH
    if inter <= 0:
        return 0.0
    areaA = max(0, (boxA[2]-boxA[0])) * max(0, (boxA[3]-boxA[1]))
    areaB = max(0, (boxB[2]-boxB[0])) * max(0, (boxB[3]-boxB[1]))
    union = areaA + areaB - inter
    return inter / float(union) if union > 0 else 0.0


@dataclass
class _Track:
    track_id: int
    bbox: Tuple[int,int,int,int]
    conf: float
    age: int = 0
    time_since_update: int = 0
    hits: int = 0


class IOUTracker:
    """
    Minimal IOU-based tracker with greedy assignment and simple aging.

    update(detections, image_size) -> List[(track_id, bbox, conf)]
    - detections: List[(bbox_xyxy, conf)]
    - image_size unused (API placeholder)
    """
    def __init__(self, max_age: int = 8, iou_threshold: float = 0.3, min_hits: int = 3):
        self.max_age = int(max_age)
        self.iou_threshold = float(iou_threshold)
        self.min_hits = int(min_hits)
        self._next_id = 1
        self._tracks: List[_Track] = []

    def update(self, detections: List[Tuple[Tuple[int,int,int,int], float]], image_size: Tuple[int,int]) \
            -> List[Tuple[int, Tuple[int,int,int,int], float]]:
        # Step 1: mark all tracks as unmatched
        for tr in self._tracks:
            tr.age += 1
            tr.time_since_update += 1

        if not detections and not self._tracks:
            return []

        unmatched_tracks = set(range(len(self._tracks)))
        unmatched_dets = set(range(len(detections)))

        # Step 2: compute IOU matrix and greedy match
        pairs: List[Tuple[float, int, int]] = []  # (iou, t_idx, d_idx)
        for ti, tr in enumerate(self._tracks):
            for di, (dbox, _) in enumerate(detections):
                iou = _iou(tr.bbox, dbox)
                if iou >= self.iou_threshold:
                    pairs.append((iou, ti, di))
        pairs.sort(reverse=True)

        for iou, ti, di in pairs:
            if ti in unmatched_tracks and di in unmatched_dets:
                # match
                self._tracks[ti].bbox = detections[di][0]
                self._tracks[ti].conf = detections[di][1]
                self._tracks[ti].time_since_update = 0
                self._tracks[ti].hits += 1
                if ti in unmatched_tracks: unmatched_tracks.remove(ti)
                if di in unmatched_dets: unmatched_dets.remove(di)

        # Step 3: create new tracks for unmatched detections
        for di in list(unmatched_dets):
            box, conf = detections[di]
            self._tracks.append(_Track(self._next_id, box, conf, age=1, time_since_update=0, hits=1))
            self._next_id += 1

        # Step 4: remove dead tracks
        self._tracks = [t for t in self._tracks if t.time_since_update <= self.max_age]

        # Step 5: output active, recently updated tracks
        out: List[Tuple[int, Tuple[int,int,int,int], float]] = []
        for t in self._tracks:
            if t.time_since_update == 0 and t.hits >= self.min_hits:
                out.append((t.track_id, t.bbox, t.conf))
        # If nothing meets min_hits yet, allow provisional outputs
        if not out:
            for t in self._tracks:
                if t.time_since_update == 0:
                    out.append((t.track_id, t.bbox, t.conf))
        return out

