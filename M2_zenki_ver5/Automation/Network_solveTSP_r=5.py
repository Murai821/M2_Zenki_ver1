#プログラムが入っているディレクトリ名をL.72「jidouka_2」から変更する必要あり

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
import os
import networkx as nx

# ２点間が繋がっていれば距離を返す関数
def distance(u, v):
    x1, y1 = positions[u]
    x2, y2 = positions[v]
    for i in range(point_num):
        if ad_list[u][v] == 1:
            return ((x1-x2)**2 + (y1-y2)**2)**0.5
    return INF

# パラメータを設定
INF = 9999
count = 0
L = 10 #SAの大きさ
d_limit = 4 #山間部避難所は rasius + d_limt より離れた場所に生成
point_num = 51 #全部で生成する点の数
center_point = (5, 5)  # 中央の点
circle_radius = 5  # 円の半径
num_points_outer = 0  # 円外の点の数
num_points_inner = point_num - num_points_outer  # 円内の点の数

# ランダムシードを設定（この値を変更することによって生成する道路網をランダム生成）
np.random.seed(45)

# 中央の点 (5, 5) を設定
central_point = np.array([center_point])

# 円内の点を生成
angles = np.random.uniform(0, 2 * np.pi, num_points_inner - 1) #num_points_inner - 1 個の点は円内にランダム生成
radii = np.random.uniform(0+0.5, circle_radius, num_points_inner - 1)
x_inner = center_point[0] + radii * np.cos(angles)
y_inner = center_point[1] + radii * np.sin(angles)
# 中央の点を先頭に追加
x_inner = np.insert(x_inner, 0, center_point[0])
y_inner = np.insert(y_inner, 0, center_point[1])
points_inner = np.vstack([x_inner, y_inner]).T


# 円外の点を中心（5, 5）の半径circle_radiusの円外でランダム生成
angles_outer = np.random.uniform(0, 2 * np.pi, num_points_outer)
radii_outer = np.random.uniform(d_limit, L/2, num_points_outer)#circle_radius + 2 から10/2だけの距離の間に
x_outer = center_point[0] + radii_outer * np.cos(angles_outer)
y_outer = center_point[1] + radii_outer * np.sin(angles_outer)
points_outer = np.vstack([x_outer, y_outer]).T

# 50個の点を合成
points = np.vstack([points_inner, points_outer])

# 円内の点のデローニー三角分割を計算
tri_inner = Delaunay(points_inner)

# 各点の隣接点を見つける
neighbors = {i: set() for i in range(len(points))}
for simplex in tri_inner.simplices:
    for i in range(3):
        neighbors[simplex[i] ].update([simplex[(i+1) % 3] , simplex[(i+2) % 3] ])

# 円外の点を円内の最も近い点に接続
for i in range(num_points_outer):
    outer_point_idx = num_points_inner  + i
    distances = np.linalg.norm(points_inner - points_outer[i], axis=1)
    closest_inner_point_idx = np.argmin(distances)
    
    # outer_point_idxがneighborsに存在しない場合は新しく追加する
    if outer_point_idx not in neighbors:
        neighbors[outer_point_idx] = set()
    
    neighbors[outer_point_idx].add(closest_inner_point_idx)
    neighbors[closest_inner_point_idx].add(outer_point_idx)



# 各点の隣接点を出力
for point_idx, adjacent_points in neighbors.items():
    print(f"Point {point_idx} is connected to points {list(adjacent_points)}")

# プロット
plt.figure(figsize=(8, 8))
plt.triplot(points_inner[:, 0], points_inner[:, 1], tri_inner.simplices, color='b')
plt.plot(points[:, 0], points[:, 1], 'o', color='r', markersize=3)

# 各点に番号を付ける
for i, (x, y) in enumerate(points):
    plt.text(x, y, str(i), fontsize=10, ha='center', va='bottom')

# 円を描画
circle = plt.Circle(center_point, circle_radius, color='g', fill=False, linestyle='--')
plt.gca().add_artist(circle)

# 円外の点と円内の最も近い点を接続する線を描画
for i in range(num_points_outer):
    outer_point_idx = num_points_inner  + i
    closest_inner_point_idx = min(neighbors[outer_point_idx])
    plt.plot([points[outer_point_idx, 0], points[closest_inner_point_idx, 0]],
             [points[outer_point_idx, 1], points[closest_inner_point_idx, 1]], 'b--')

plt.title("Delaunay Triangulation with Central Point (5, 5)")
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.gca().set_aspect('equal', adjustable='box')

# カレントディレクトリ取得
current_directory = os.getcwd()
print("現在のカレントディレクトリ:", current_directory)

# 新しいファイルのパスを指定（このプログラムが入っているディレクトリ名を「jidouka_2」から変更!!!）
points_file_path = os.path.join(current_directory, "Automation/pythonfile/points.txt")
adjacency_matrix_file_path = os.path.join(current_directory, "Automation/pythonfile/adjacency_matrix.txt")
plot_image_path = os.path.join(current_directory, "Automation/pythonfile/network_plot.png")
jyunkairo_image_path = os.path.join(current_directory, "Automation/pythonfile/jyunkairo_plot.png")
tsp_result_file_path = os.path.join(current_directory, "Automation/pythonfile/tsp_result.txt")

# plot_image_path = os.path.join(network_directory, "network_plot.png")

# テキストファイルに座標を保存
try:
    with open(points_file_path, "w") as f:
        for point in points:
            f.write("{:.6f} {:.6f}\n".format(point[0], point[1]))
    print("points.txtに座標が正常に保存されました。")
except IOError as e:
    print(f"ファイルの書き込み中にエラーが発生しました: {e}")

# 隣接行列を生成
adj_matrix = np.zeros((len(points), len(points)), dtype=int)
for i, adjacent_points in neighbors.items():
    for j in adjacent_points:
        adj_matrix[i, j] = 1

# 隣接行列をテキストファイルに保存
try:
    with open(adjacency_matrix_file_path, "w") as f:
        for row in adj_matrix:
            f.write(" ".join(map(str, row)) + "\n")
    print("adjacency_matrix.txtに隣接行列が正常に保存されました。")
except IOError as e:
    print(f"ファイルの書き込み中にエラーが発生しました: {e}")

# 画像を保存
plt.savefig(plot_image_path)

# ラベルを初期化する（リセットする）方法
plt.clf()  # 図をクリアする
plt.close()  # 図を閉じる

print(f"プロット画像が {plot_image_path} に保存されました。")

""""上で作成した道路ネットワークにおいて巡回セールスマン問題を解くプログラム"""

# points.txtに書き込む座標をpositionsに格納
positions = {i: (point[0], point[1]) for i, point in enumerate(points)}

# 隣接行列 ad_list を生成
ad_list = [[INF]*len(points) for _ in range(len(points))]
for i, adjacent_points in neighbors.items():
    for j in adjacent_points:
        ad_list[i][j] = 1

# グラフを作成
G = nx.Graph()
G.add_nodes_from(positions.keys())
for u in positions:
    for v in positions:
        if u < v:
            G.add_edge(u, v, weight=distance(u, v))
            count += count + 1


# 巡回セールスマン問題を解く
tsp = list(nx.algorithms.approximation.traveling_salesman_problem(G))

# TSP結果をファイルに保存
try:
    with open(tsp_result_file_path, "w") as f:
        # C言語の配列形式で保存
        f.write("int tsp_route[] = {")
        f.write(", ".join(map(str, tsp)))
        f.write("};\n")
    print("tsp_result.txtにTSP結果が正常に保存されました。")
except IOError as e:
    print(f"ファイルの書き込み中にエラーが発生しました: {e}")

# 解を表示
print("最短距離の順序：", tsp)
print("最短距離：", sum(distance(u, v) for u, v in zip(tsp, tsp[1:]+tsp[:1])))
print("要素数：",len(tsp))

# グラフを可視化
pos = positions
nx.draw(G, pos=pos, node_size=90)
nx.draw_networkx_labels(G, pos=pos, font_size=8)  # ラベルのフォントサイズを8に設定
nx.draw_networkx_edges(G, pos=pos, edgelist=[(u, v) for u, v in zip(tsp, tsp[1:]+tsp[:1])], edge_color='r')
# 画像を保存
plt.savefig(jyunkairo_image_path)
