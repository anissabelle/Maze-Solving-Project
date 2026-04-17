import argparse
import random
import sys
from pathlib import Path

import numpy as np

TIER_SIDE_RANGE = {"small": (10, 50), "medium": (100, 250), "large": (500, 1000)}


def create_perfect_maze(dim: int) -> np.ndarray:
    maze = np.ones((dim * 2 + 1, dim * 2 + 1), dtype=int)
    maze[1, 1] = 0
    stack = [(0, 0)]
    while stack:
        x, y = stack[-1]
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < dim and 0 <= ny < dim and maze[2 * nx + 1, 2 * ny + 1] == 1:
                maze[2 * nx + 1, 2 * ny + 1] = 0
                maze[2 * x + 1 + dx, 2 * y + 1 + dy] = 0
                stack.append((nx, ny))
                break
        else:
            stack.pop()
    maze[1, 0] = 0
    maze[-2, -1] = 0
    return maze


def create_imperfect_maze(dim: int, extra_wall_removals: float) -> np.ndarray:
    maze = create_perfect_maze(dim)
    candidates = []
    for r in range(1, maze.shape[0] - 1):
        for c in range(1, maze.shape[1] - 1):
            if maze[r, c] != 1:
                continue
            if maze[r - 1, c] == 0 and maze[r + 1, c] == 0:
                candidates.append((r, c))
            elif maze[r, c - 1] == 0 and maze[r, c + 1] == 0:
                candidates.append((r, c))
    k = min(int(len(candidates) * extra_wall_removals), len(candidates))
    if k > 0:
        for r, c in random.sample(candidates, k):
            maze[r, c] = 0
    return maze


def maze_nxn(n: int, kind: str, extra_wall_removals: float) -> np.ndarray:
    if n < 3:
        raise ValueError("maze side length n must be >= 3")

    if n % 2 == 1:
        dim = (n - 1) // 2
        return (
            create_perfect_maze(dim)
            if kind == "perfect"
            else create_imperfect_maze(dim, extra_wall_removals)
        )

    inner = n - 1
    dim = (inner - 1) // 2
    core = (
        create_perfect_maze(dim)
        if kind == "perfect"
        else create_imperfect_maze(dim, extra_wall_removals)
    )
    out = np.ones((n, n), dtype=int)
    out[:inner, :inner] = core
    out[inner - 2, inner] = 0
    return out


def maze_text(maze: np.ndarray) -> str:
    return "\n".join(
        "".join("#" if maze[r, c] else "." for c in range(maze.shape[1]))
        for r in range(maze.shape[0])
    ) + "\n"


def write_batch(
    output_dir: Path,
    kind: str,
    tier: str,
    count: int,
    extra_wall_removals: float,
) -> int:
    lo, hi = TIER_SIDE_RANGE[tier]
    output_dir.mkdir(parents=True, exist_ok=True)
    for i in range(1, count + 1):
        n = random.randint(lo, hi)
        maze = maze_nxn(n, kind, extra_wall_removals)
        path = output_dir / f"{kind}_{tier}_{i}.txt"
        path.write_text(maze_text(maze), encoding="ascii")
    return count


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate maze matrix files.")
    p.add_argument("kind", choices=("perfect", "imperfect"))
    p.add_argument("tier", choices=("small", "medium", "large"))
    p.add_argument("count", type=int, help="1..1000")
    p.add_argument("-o", "--output-dir", type=Path, default=Path("mazes"))
    p.add_argument("--extra-wall-removals", type=float, default=0.05)
    p.add_argument("--seed", type=int, default=None)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if not 1 <= args.count <= 1000:
        print("error: count must be 1..1000", file=sys.stderr)
        sys.exit(1)
    if not 0 <= args.extra_wall_removals <= 1:
        print("error: --extra-wall-removals must be in [0, 1]", file=sys.stderr)
        sys.exit(1)
    if args.seed is not None:
        random.seed(args.seed)

    n = write_batch(
        output_dir=args.output_dir,
        kind=args.kind,
        tier=args.tier,
        count=args.count,
        extra_wall_removals=args.extra_wall_removals,
    )
    print(f"Wrote {n} file(s) to {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
