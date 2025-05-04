# file: loader.py
import csv, random
from vpython import vector
import config
from agent import Agent
import scenes


def load_agents(csv_path: str = "node.csv") -> list:
    """
    CSVを読み込み、Agentインスタンスを作成し、隣接リストを設定します。

    Parameters:
        csv_path (str): node.csv のパス

    Returns:
        List[Agent]: 初期化済みエージェントのリスト
    """
    agents = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for k, row in enumerate(reader):
            i, j = divmod(k, config.COLS)
            z = random.uniform(
                config.minZ + config.agent_length/2,
                config.maxZ - config.agent_length/2
            )
            agents.append(
                Agent(
                    i, j,
                    float(row["x"]),
                    float(row["y"]),
                    z,
                    row["id"]
                )
            )
    # 隣接設定
    for ag in agents:
        i, j = ag.i, ag.j
        nbrs = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
        # チェックボード型近接
        if i % 2 == 0:
            nbrs += [(i-1, j-1), (i+1, j-1)]
        else:
            nbrs += [(i-1, j+1), (i+1, j+1)]
        # 範囲内のみ有効なインデックス
        ag.neighbors = [ni*config.COLS + nj for ni, nj in nbrs
                        if 0 <= ni < config.ROWS and 0 <= nj < config.COLS]
    return agents


def compute_camera_centers(agents: list) -> None:
    """
    エージェント位置に応じてシーンの中心と2Dビュー範囲を設定します。

    Parameters:
        agents (list of Agent)
    """
    xs = [a.x for a in agents]
    ys = [a.y for a in agents]
    minX, maxX = min(xs), max(xs)
    minY, maxY = min(ys), max(ys)
    centerX = (minX + maxX) / 2
    centerY = (minY + maxY) / 2
    span = max(maxX - minX, maxY - minY)

    # 3Dシーン中心
    scenes.scene3d.center = vector(
        centerX,
        centerY,
        (config.minZ + config.maxZ) / 2
    )
    # 2Dシーン中心と範囲
    scenes.scene2d.center = vector(centerX, centerY, 0)
    scenes.scene2d.range = span * 0.4
