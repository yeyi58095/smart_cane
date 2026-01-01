import math
from collections import deque
from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid


class GoalSnapper:
    """
    Snap a (x,y) in map frame to nearest reachable/free cell using global_costmap OccupancyGrid.
    """

    def __init__(
        self,
        occ_th: int = 50,              # >= occ_th => treated as occupied
        unknown_is_obstacle: bool = True,
        clearance_m: float = 0.30,     # safety radius around snapped cell
        search_radius_m: float = 2.0,  # max search distance
    ):
        self.grid: Optional[OccupancyGrid] = None
        self.occ_th = int(occ_th)
        self.unknown_is_obstacle = bool(unknown_is_obstacle)
        self.clearance_m = float(clearance_m)
        self.search_radius_m = float(search_radius_m)

    def set_grid(self, grid: OccupancyGrid):
        self.grid = grid

    # ---------- helpers ----------
    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.grid is None:
            return None
        info = self.grid.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        cx = int(math.floor((x - ox) / res))
        cy = int(math.floor((y - oy) / res))
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return cx, cy

    def _cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        info = self.grid.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        # center of cell
        x = ox + (cx + 0.5) * res
        y = oy + (cy + 0.5) * res
        return x, y

    def _idx(self, cx: int, cy: int) -> int:
        info = self.grid.info
        return cy * info.width + cx

    def _is_free_cell(self, cx: int, cy: int) -> bool:
        """Check cell is free w.r.t occupancy threshold and unknown policy."""
        assert self.grid is not None
        v = self.grid.data[self._idx(cx, cy)]
        if v == -1:
            return not self.unknown_is_obstacle
        return v < self.occ_th

    def _is_free_with_clearance(self, cx: int, cy: int) -> bool:
        """Check a disk neighborhood is all free."""
        assert self.grid is not None
        info = self.grid.info
        res = info.resolution
        r_cells = int(math.ceil(self.clearance_m / res))
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy > r_cells * r_cells:
                    continue
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    return False
                if not self._is_free_cell(nx, ny):
                    return False
        return True

    # ---------- main API ----------
    def snap(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """
        Return snapped (x,y) in map frame, or None if no suitable free cell found.
        """
        if self.grid is None:
            return None

        start = self._world_to_cell(x, y)
        if start is None:
            return None

        sx, sy = start
        info = self.grid.info

        # if already good
        if self._is_free_with_clearance(sx, sy):
            return self._cell_to_world(sx, sy)

        # BFS within radius
        res = info.resolution
        max_cells = int(math.ceil(self.search_radius_m / res))
        q = deque()
        q.append((sx, sy))
        visited = set()
        visited.add((sx, sy))

        # 4-neighborhood is fine (faster + stable)
        neigh = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while q:
            cx, cy = q.popleft()

            # stop if too far
            if abs(cx - sx) > max_cells or abs(cy - sy) > max_cells:
                continue

            if self._is_free_with_clearance(cx, cy):
                return self._cell_to_world(cx, cy)

            for dx, dy in neigh:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    continue
                if (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                q.append((nx, ny))

        return None
