#プログラムが入っているディレクトリ名をL.72「jidouka_2」から変更あり

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

INF = 9999
count = 0
point_num = 51 #全部で生成する点の数

"""ランダム生成した点をデローニー三角分割法により結び道路ネットワークを作成するプログラム"""
# ランダムシードを設定（この値を変更することによって生成する道路網をランダム生成）
np.random.seed(45)

# 一様分布した50個の点を生成
num_points = 50
points = np.random.rand(num_points, 2) * 10  # 0から10の範囲で一様分布

# 中央の点 (5, 5) を追加
central_point = np.array([[5, 5]])
points = np.vstack([central_point, points])  # 中央の点を先頭に追加

# デローニー三角分割を計算
tri = Delaunay(points)

# 各点の隣接点を見つける
neighbors = {i: set() for i in range(len(points))}
for simplex in tri.simplices:
    for i in range(3):
        neighbors[simplex[i]].update([simplex[(i+1)%3], simplex[(i+2)%3]])

# 各点の隣接点を出力
for point_idx, adjacent_points in neighbors.items():
    print(f"Point {point_idx} is connected to points {list(adjacent_points)}")

# プロット
plt.figure(figsize=(8, 8))
plt.triplot(points[:,0], points[:,1], tri.simplices, color='b')
plt.plot(points[:,0], points[:,1], 'o', color='r',markersize=3)

# 各点に番号を付ける
for i, (x, y) in enumerate(points):
    plt.text(x, y, str(i), fontsize=13, ha='center', va='bottom')

plt.title("Delaunay Triangulation with Central Point (5, 5)")
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.gca().set_aspect('equal', adjustable='box')
#plt.grid(True)
#plt.show()

#　カレントディレクトリ取得
current_directory = os.getcwd()
print("現在のカレントディレクトリ:", current_directory)

# 新しいファイルのパスを指定（このプログラムが入っているディレクトリ名を「jidouka_2」から変更!!!）
points_file_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/points.txt")
adjacency_matrix_file_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/adjacency_matrix.txt")
plot_image_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/network_plot.png")
jyunkairo_image_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/jyunkairo_plot.png")
tsp_result_file_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/tsp_result.txt")

#plot_image_path = os.path.join(network_directory, "network_plot.png")

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

# 生成された点のみをプロットし、ディレクトリにpngとして保存
plt.figure(figsize=(8, 8))
plt.plot(points[:,0], points[:,1], 'o', color='r', markersize=3)

# 各点に番号を付ける
for i, (x, y) in enumerate(points):
    plt.text(x, y, str(i), fontsize=13, ha='center', va='bottom')

plt.title("Generated Points")
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.gca().set_aspect('equal', adjustable='box')
generated_points_image_path = os.path.join(current_directory, "Automation_nodrone/pythonfile/generated_points.png")
plt.savefig(generated_points_image_path)
plt.clf()  # 図をクリアする
plt.close()  # 図を閉じる

print(f"生成された点のプロット画像が {generated_points_image_path} に保存されました。")

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
nx.draw(G, pos=pos, with_labels=True)
nx.draw_networkx_edges(G, pos=pos, edgelist=[(u, v) for u, v in zip(tsp, tsp[1:]+tsp[:1])], edge_color='r')
# 画像を保存
plt.savefig(jyunkairo_image_path)
#plt.show()
