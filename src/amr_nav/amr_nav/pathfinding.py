#!/usr/bin/env python3
"""
Pure pathfinding algorithms for occupancy grid navigation.

No ROS dependencies — operates on numpy arrays only.

Algorithms:
  1. occupancy_to_grid: Convert OccupancyGrid data to binary grid
  2. find_next_target: Multi-source Dijkstra to find most-unexplored cell
  3. astar_wall_penalty: A* with wall proximity penalty
  4. cluster_path: Reduce path to cluster waypoints
"""

import heapq
import numpy as np


def occupancy_to_grid(data, width, height, wall_threshold=10):
    """Convert flat OccupancyGrid data to a 2D binary grid.

    Args:
        data: Flat list of occupancy values (-1=unknown, 0-100=probability).
        width: Grid width.
        height: Grid height.
        wall_threshold: Values >= this are considered occupied.

    Returns:
        2D numpy array (height x width), 0=free, 1=occupied.
    """
    if width == 0 or height == 0:
        return np.zeros((height, width), dtype=np.int8)

    arr = np.array(data, dtype=np.int16).reshape((height, width))
    grid = np.zeros((height, width), dtype=np.int8)
    grid[(arr == -1) | (arr >= wall_threshold)] = 1
    return grid


def find_next_target(grid, visited_cells, distance_threshold=35):
    """Multi-source Dijkstra to find the most-unexplored reachable cell.

    All visited cells start as sources with cost=0. Dijkstra expands
    through FREE cells. The highest-cost reachable FREE cell that is
    NOT in the visited set is the target (furthest from all visited
    areas = most unexplored).

    Args:
        grid: 2D numpy array (0=free, 1=occupied).
        visited_cells: Set of (row, col) tuples that have been visited.
        distance_threshold: Max cost to consider.

    Returns:
        (row, col) of the target cell, or None if no target found.
    """
    if not visited_cells:
        return None

    height, width = grid.shape
    dist = np.full((height, width), np.inf)

    pq = []
    for r, c in visited_cells:
        if 0 <= r < height and 0 <= c < width and grid[r, c] == 0:
            dist[r, c] = 0
            heapq.heappush(pq, (0, r, c))

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    best_cell = None
    best_cost = -1

    while pq:
        cost, r, c = heapq.heappop(pq)

        if cost > dist[r, c]:
            continue
        if cost > distance_threshold:
            continue

        if cost > best_cost and (r, c) not in visited_cells:
            best_cost = cost
            best_cell = (r, c)

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] == 0:
                new_cost = cost + 1
                if new_cost < dist[nr, nc]:
                    dist[nr, nc] = new_cost
                    heapq.heappush(pq, (new_cost, nr, nc))

    return best_cell


def _precompute_wall_proximity(grid, radius):
    """Vectorised wall proximity using integral image.

    Returns a 2D array where each cell contains the count of occupied
    cells within a (2*radius+1) square neighbourhood.
    """
    binary = (grid == 1).astype(np.int64)
    h, w = binary.shape
    padded = np.pad(binary, radius, mode='constant', constant_values=0)

    # Summed area table with one-row/col zero border
    sat = np.zeros((h + 2 * radius + 1, w + 2 * radius + 1), dtype=np.int64)
    sat[1:, 1:] = np.cumsum(np.cumsum(padded, axis=0), axis=1)

    k = 2 * radius + 1
    result = sat[k:, k:] - sat[:h, k:] - sat[k:, :w] + sat[:h, :w]
    return result.astype(np.int32)


def astar_wall_penalty(grid, start, goal, wall_penalty=5, wall_cost=200):
    """A* pathfinding with wall proximity penalty.

    For each candidate cell, the number of occupied cells within a
    square neighbourhood of radius *wall_penalty* is counted and
    multiplied by *wall_cost* to increase the movement cost, naturally
    steering paths away from walls.

    Args:
        grid: 2D numpy array (0=free, 1=occupied).
        start: (row, col) start position.
        goal: (row, col) goal position.
        wall_penalty: Neighbourhood scan radius for wall detection.
        wall_cost: Cost multiplier per nearby wall cell.

    Returns:
        List of (row, col) from start to goal, or empty list if no path.
    """
    height, width = grid.shape
    sr, sc = start
    gr, gc = goal

    if not (0 <= sr < height and 0 <= sc < width):
        return []
    if not (0 <= gr < height and 0 <= gc < width):
        return []
    if grid[sr, sc] == 1 or grid[gr, gc] == 1:
        return []
    if start == goal:
        return [start]

    wall_prox = _precompute_wall_proximity(grid, wall_penalty)

    def heuristic(r, c):
        return abs(r - gr) + abs(c - gc)

    open_set = [(heuristic(sr, sc), 0, sr, sc)]
    g_score = {(sr, sc): 0}
    came_from = {}
    closed = set()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while open_set:
        _f, g, r, c = heapq.heappop(open_set)

        if (r, c) in closed:
            continue
        closed.add((r, c))

        if r == gr and c == gc:
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            path.reverse()
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if grid[nr, nc] == 1 or (nr, nc) in closed:
                continue

            move_cost = 1 + int(wall_prox[nr, nc]) * wall_cost
            new_g = g + move_cost

            if new_g < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = new_g
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_set, (new_g + heuristic(nr, nc),
                                          new_g, nr, nc))

    return []


def cluster_path(path, cluster_distance=6):
    """Reduce a dense path to cluster waypoints.

    Keeps only waypoints that are at least *cluster_distance* grid cells
    apart (Manhattan distance). Always includes start and end points.

    Args:
        path: List of (row, col) tuples (dense path from A*).
        cluster_distance: Minimum spacing between waypoints.

    Returns:
        List of (row, col) cluster waypoints.
    """
    if len(path) <= 1:
        return list(path)

    clusters = [path[0]]

    for point in path[1:]:
        lr, lc = clusters[-1]
        pr, pc = point
        if abs(pr - lr) + abs(pc - lc) >= cluster_distance:
            clusters.append(point)

    if clusters[-1] != path[-1]:
        clusters.append(path[-1])

    return clusters
