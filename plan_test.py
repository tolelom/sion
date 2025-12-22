# plan_test.py

from typing import List, Tuple
import json

from map_loader import load_map, CellType, cell_to_world
from astar import astar_normal, astar_enemy
from path_utils import smooth_path_cells, inflate_map, attach_charge_segment, bresenham_line



def print_map_with_path(
    cells: List[str],
    path: List[Tuple[int, int]],
    title: str,
) -> None:
    """
    문자 맵(cells) 위에 path를 덮어써서 출력.
    - 제어용 path는 웨이포인트 위주이지만,
      시각화할 때는 각 구간을 Bresenham으로 채워서
      실제 로봇이 달릴 '연속 경로'처럼 보이게 만든다.
    """
    print("\n" + title)
    char_grid = [list(row) for row in cells]

    if path:
        # 구간마다 선분을 채워 넣음
        for i in range(len(path) - 1):
            (x0, y0) = path[i]
            (x1, y1) = path[i + 1]
            seg = bresenham_line((x0, y0), (x1, y1))
            for (sx, sy) in seg:
                if char_grid[sy][sx] in (".",):
                    char_grid[sy][sx] = "*"

        # 시작/목표는 따로 강조
        sx, sy = path[0]
        gx, gy = path[-1]
        char_grid[sy][sx] = "S"
        char_grid[gy][gx] = "E"

    for row in reversed(char_grid):
        print("".join(row))



def plan_path_for_goal(
    m_orig,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    is_enemy_goal: bool,
    inflate_radius: int = 2,
    min_charge_cells: int = 8,
):
    """
    프로젝트 요구사항에 맞는 '최종 경로 생성' 함수.

    - is_enemy_goal == False:
        -> 최단 경로 (대각선 포함) + smoothing 만 적용
    - is_enemy_goal == True:
        -> ENEMY용 A* (회전 최소화) + smoothing
           + 마지막은 '직선 돌진 구간'을 강제로 포함
    """
    # 1) inflate 된 맵으로 경로계획 (충돌 여유 확보용)
    m = inflate_map(m_orig, radius_cells=inflate_radius)

    gx, gy = goal_cell
    use_enemy_mode = is_enemy_goal


    # 2) A* 선택
    if use_enemy_mode:
        raw_path = astar_enemy(m, start_cell, goal_cell)
    else:
        raw_path = astar_normal(m, start_cell, goal_cell)

    if raw_path is None:
        return None, use_enemy_mode

    # 3) smoothing
    smooth_path = smooth_path_cells(m, raw_path)

    # 4) 적 모드라면 돌진 구간 붙이기
    if use_enemy_mode:
        final_path = attach_charge_segment(
            m_orig, smooth_path, min_charge_cells=min_charge_cells
        )
    else:
        final_path = smooth_path

    return final_path, use_enemy_mode


def main():
    m_orig = load_map("map_smallroom_60x60.json")

    if m_orig.start is None:
        raise RuntimeError("Start(S) not found in map")

    start = m_orig.start

    # --------------------------
    # ★ 여기 부분이 나중에 "서버에서 목표 좌표 받는 자리"
    # --------------------------
    # 예시1: 지금처럼 맵에 있는 적(E)로 가고 싶으면:
    if not m_orig.enemies:
        raise RuntimeError("Enemy(E) not found in map")
    goal_cell = m_orig.enemies[0]    # (50, 49) 같은 좌표
    is_enemy_goal = False             # <- 나중에 서버에서 true/false 넘어옴

    # 예시2: 일반 길 한가운데 좌표로 이동시키고 싶으면:
    # goal_cell = (30, 20)
    # is_enemy_goal = False

    print("Start cell:", start)
    print("Goal  cell:", goal_cell)
    print("is_enemy_goal:", is_enemy_goal)

    # 1) 경로 생성
    path, used_enemy_mode = plan_path_for_goal(
        m_orig,
        start_cell=start,
        goal_cell=goal_cell,
        is_enemy_goal=is_enemy_goal,
        inflate_radius=2,      # 필요하면 1~3 사이에서 튜닝
        min_charge_cells=8,    # 마지막 돌진 직선 최소 8셀(0.4m) 정도
    )

    if path is None:
        print("No path found")
        return

    print("Used enemy mode:", used_enemy_mode)
    print("Final path length (cells):", len(path))

    # 2) 원본 맵 불러와서 시각화
    with open("map_smallroom_60x60.json", "r") as f:
        raw = json.load(f)
        cells = raw["cells"]

    print_map_with_path(cells, path, "Final path (with charge if enemy)")

    # 3) 월드 좌표로 찍기 (나중에 모터 제어에 사용)
    print("\nFinal path (world coordinates, meters):")
    for (cx, cy) in path:
        wx, wy = cell_to_world(cx, cy, m_orig.resolution)
        print(f"  cell ({cx:2d},{cy:2d}) -> world ({wx:5.2f}, {wy:5.2f})")


if __name__ == "__main__":
    main()
