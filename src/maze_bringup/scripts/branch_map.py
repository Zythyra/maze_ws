"""Prove a saved branch is a fully observed pocket with only one entrance."""
from dataclasses import dataclass, field
import cv2
import numpy as np

from polyline_core import Frame2D, direction


@dataclass(frozen=True)
class BranchProof:
    dead_end: bool
    reason: str
    cells: int = 0
    region: np.ndarray | None = field(default=None, repr=False, compare=False)


class BranchMap:
    def __init__(self, cells, resolution, origin, yaw=0.):
        self.cells = np.asarray(cells, dtype=np.int16).copy()
        self.resolution = float(resolution)
        self.frame = Frame2D(np.asarray(origin, dtype=float), yaw)
        self.free = ((self.cells >= 0) & (self.cells < 65)).astype(np.uint8)
        self.unknown_border = cv2.dilate((self.cells < 0).astype(np.uint8),
                                        np.ones((3, 3), dtype=np.uint8)).astype(bool)
        self.closed_regions = []

    def remember(self, proof):
        if proof.dead_end and proof.region is not None:
            self.closed_regions.append(proof)

    def enclosing_proof(self, point):
        row, col = self.cell(point)
        if 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]:
            for proof in self.closed_regions:
                if proof.region[row, col]:return proof
        return None

    def cell(self, point):
        col, row = np.floor(self.frame.inverse(point) / self.resolution).astype(int)
        return int(row), int(col)

    def value(self, point):
        row, col = self.cell(point)
        if not (0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]):
            return -1
        return int(self.cells[row, col])

    def check(self, point, heading):
        point = np.asarray(point, dtype=float)
        enclosing = self.enclosing_proof(point)
        if enclosing is not None:
            return BranchProof(True, 'junction is inside a fully observed closed pocket',
                               enclosing.cells, enclosing.region)
        axis, normal = direction(heading), direction(heading + 1)
        # Beyond the parent corridor's side wall, but before a distant tip.
        # Both ends of this artificial cut must terminate in measured walls.
        mouth = point + axis * .38
        seed = mouth + axis * max(.10, self.resolution * 4)
        for distance in np.arange(0., np.linalg.norm(seed - point) + self.resolution / 2,
                                  self.resolution / 2):
            value = self.value(point + axis * distance)
            if not 0 <= value < 65:
                return BranchProof(False, 'entrance is not fully observed free space')
        ends = []
        for sign in (-1, 1):
            for distance in np.arange(.10, .61, self.resolution / 2):
                sample = mouth + normal * sign * distance
                value = self.value(sample)
                if value < 0:
                    return BranchProof(False, 'mouth side wall is unknown')
                if value >= 65:
                    ends.append(self.cell(sample))
                    break
            else:
                return BranchProof(False, 'no measured narrow entrance')
        region = self.free.copy()
        cv2.line(region, ends[0][::-1], ends[1][::-1], 0, thickness=3)
        source_cell, seed_cell = self.cell(point), self.cell(seed)
        if not region[seed_cell] or not region[source_cell]:
            return BranchProof(False, 'entrance cut overlaps a probe')
        # Raw free space, with eight-neighbour connectivity. Do not inflate
        # obstacles: inflation could erase a narrow real door and invent a
        # dead end. Even a diagonal connection to unknown space prevents proof.
        area, _, _, _ = cv2.floodFill(region, None, seed_cell[::-1], 2, flags=8)
        reached = region == 2
        if reached[source_cell]:
            return BranchProof(False, 'branch reconnects to the parent corridor')
        if (np.any(reached[0]) or np.any(reached[-1]) or
                np.any(reached[:, 0]) or np.any(reached[:, -1])):
            return BranchProof(False, 'branch reaches the map boundary')
        if np.any(reached & self.unknown_border):
            return BranchProof(False, 'branch still borders unknown space')
        return BranchProof(True, 'fully observed pocket; entrance is its only connection', area, reached)
