#!/usr/bin/env python3
"""Convert a 3D LiDAR point cloud (.ply) to a Nav2 occupancy map (.yaml + .pgm)."""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml

# Match mapping.launch.py / pointcloud_aggregator defaults
DEFAULT_HEIGHT_MIN = -1.0
DEFAULT_HEIGHT_MAX = 3.0
DEFAULT_RESOLUTION = 0.05
DEFAULT_PADDING = 1.0
DEFAULT_INFLATION = 0.15  # meters — slight obstacle padding for Nav2
FREE_VALUE = 254
OCCUPIED_VALUE = 0
UNKNOWN_VALUE = 205


def load_points(ply_path: Path) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(str(ply_path))
    if cloud.is_empty():
        raise ValueError(f"No points found in {ply_path}")
    return np.asarray(cloud.points, dtype=np.float64)


def filter_height(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered = points[mask]
    if filtered.size == 0:
        raise ValueError(
            f"No points remain after height filter [{z_min}, {z_max}]. "
            "Try widening --height-min / --height-max."
        )
    return filtered


def world_to_grid(
    points_xy: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    height: int,
) -> np.ndarray:
    cols = np.floor((points_xy[:, 0] - origin_x) / resolution).astype(np.int32)
    rows = height - 1 - np.floor((points_xy[:, 1] - origin_y) / resolution).astype(np.int32)
    return np.stack([rows, cols], axis=1)


def mark_reachable_free(
    grid: np.ndarray,
    seed_row: int,
    seed_col: int,
) -> None:
    """Flood-fill free space from a seed; falls back to all non-occupied cells."""
    if not (0 <= seed_row < grid.shape[0] and 0 <= seed_col < grid.shape[1]):
        grid[grid != OCCUPIED_VALUE] = FREE_VALUE
        return

    if grid[seed_row, seed_col] == OCCUPIED_VALUE:
        # Origin often sits on/near lidar returns — search for a nearby seed.
        found = False
        for radius in range(1, max(grid.shape)):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue
                    nr, nc = seed_row + dr, seed_col + dc
                    if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
                        if grid[nr, nc] == UNKNOWN_VALUE:
                            seed_row, seed_col = nr, nc
                            found = True
                            break
                if found:
                    break
            if found:
                break
        if not found:
            grid[grid != OCCUPIED_VALUE] = FREE_VALUE
            return

    queue: deque[tuple[int, int]] = deque([(seed_row, seed_col)])
    grid[seed_row, seed_col] = FREE_VALUE
    filled = 1

    while queue:
        row, col = queue.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = row + dr, col + dc
            if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
                if grid[nr, nc] == UNKNOWN_VALUE:
                    grid[nr, nc] = FREE_VALUE
                    filled += 1
                    queue.append((nr, nc))

    if filled == 0:
        grid[grid != OCCUPIED_VALUE] = FREE_VALUE


def ply_to_occupancy_grid(
    points: np.ndarray,
    resolution: float = DEFAULT_RESOLUTION,
    padding: float = DEFAULT_PADDING,
    inflation: float = DEFAULT_INFLATION,
    seed_xy: tuple[float, float] = (0.0, 0.0),
) -> tuple[np.ndarray, float, float]:
    min_x = float(points[:, 0].min()) - padding
    max_x = float(points[:, 0].max()) + padding
    min_y = float(points[:, 1].min()) - padding
    max_y = float(points[:, 1].max()) + padding

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    if width < 1 or height < 1:
        raise ValueError("Computed map size is empty; check point cloud bounds.")

    grid = np.full((height, width), UNKNOWN_VALUE, dtype=np.uint8)

    cells = world_to_grid(points[:, :2], min_x, min_y, resolution, height)
    valid = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < height)
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < width)
    )
    cells = cells[valid]
    grid[cells[:, 0], cells[:, 1]] = OCCUPIED_VALUE

    if inflation > 0:
        radius_cells = max(1, int(round(inflation / resolution)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1)
        )
        occupied = (grid == OCCUPIED_VALUE).astype(np.uint8)
        grid[cv2.dilate(occupied, kernel) > 0] = OCCUPIED_VALUE

    seed_col = int(np.floor((seed_xy[0] - min_x) / resolution))
    seed_row = height - 1 - int(np.floor((seed_xy[1] - min_y) / resolution))
    mark_reachable_free(grid, seed_row, seed_col)

    return grid, min_x, min_y


def write_nav2_map(
    grid: np.ndarray,
    output_prefix: Path,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[Path, Path]:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix('.pgm')
    yaml_path = output_prefix.with_suffix('.yaml')

    if not cv2.imwrite(str(pgm_path), grid):
        raise RuntimeError(f"Failed to write {pgm_path}")

    metadata = {
        'image': pgm_path.name,
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin_x), float(origin_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with yaml_path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(metadata, handle, default_flow_style=None, sort_keys=False)

    return yaml_path, pgm_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Convert a 3D LiDAR .ply dump to a Nav2 occupancy map (.yaml + .pgm).'
    )
    parser.add_argument('ply', type=Path, help='Input .ply point cloud')
    parser.add_argument(
        '-o',
        '--output',
        type=Path,
        help='Output path prefix without extension (default: same name/dir as input .ply)',
    )
    parser.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument('--height-min', type=float, default=DEFAULT_HEIGHT_MIN)
    parser.add_argument('--height-max', type=float, default=DEFAULT_HEIGHT_MAX)
    parser.add_argument('--padding', type=float, default=DEFAULT_PADDING)
    parser.add_argument('--inflation', type=float, default=DEFAULT_INFLATION)
    parser.add_argument(
        '--seed-x',
        type=float,
        default=0.0,
        help='World X for flood-fill free-space seed (default: 0)',
    )
    parser.add_argument(
        '--seed-y',
        type=float,
        default=0.0,
        help='World Y for flood-fill free-space seed (default: 0)',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ply_path = args.ply.resolve()
    if not ply_path.is_file():
        raise SystemExit(f'Input file not found: {ply_path}')

    output_prefix = (args.output or ply_path.with_suffix('')).resolve()

    points = load_points(ply_path)
    filtered = filter_height(points, args.height_min, args.height_max)
    grid, origin_x, origin_y = ply_to_occupancy_grid(
        filtered,
        resolution=args.resolution,
        padding=args.padding,
        inflation=args.inflation,
        seed_xy=(args.seed_x, args.seed_y),
    )
    yaml_path, pgm_path = write_nav2_map(
        grid, output_prefix, origin_x, origin_y, args.resolution
    )

    occupied = int(np.sum(grid == OCCUPIED_VALUE))
    free = int(np.sum(grid == FREE_VALUE))
    unknown = int(np.sum(grid == UNKNOWN_VALUE))
    print(f'Input points: {len(points)} (after height filter: {len(filtered)})')
    print(f'Map size: {grid.shape[1]} x {grid.shape[0]} px @ {args.resolution} m/px')
    print(f'Origin: [{origin_x:.3f}, {origin_y:.3f}]')
    print(f'Cells — occupied: {occupied}, free: {free}, unknown: {unknown}')
    print(f'Wrote {yaml_path}')
    print(f'Wrote {pgm_path}')


if __name__ == '__main__':
    main()
