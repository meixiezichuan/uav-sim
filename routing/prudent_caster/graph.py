
import threading
from collections import defaultdict, deque

class Graph:
    def __init__(self, env):
        self.env = env
        self.adjList = defaultdict(list)
        self.entry_life_time = 2 * 1e6  # unit: us (1s)
        self.root = None
        self.nodeTime = defaultdict()
        self.lock = threading.RLock()

    def add_node(self, n, t):
        with self.lock:
            self.nodeTime[n] = t

    def add_edge(self, v1, v2):
        if v1 == v2:
            return
        with self.lock:
            if v2 not in self.adjList[v1]:
                self.adjList[v1].append(v2)
            if v1 not in self.adjList[v2]:
                self.adjList[v2].append(v1)

    def remove_edge(self, v1, v2):
        with self.lock:
            if v2 in self.adjList[v1]:
                self.adjList[v1].remove(v2)
            if v1 in self.adjList[v2]:
                self.adjList[v2].remove(v1)

    def get_nodes(self):
         return self.adjList.keys()

    def find_neighbor(self, node):
        """返回节点的所有邻居"""
        return self.adjList.get(node, [])

    def path_exists_in_tree(self, current_node, path):
        """在树结构中递归检查路径是否存在"""
        if path is None:
            return False
        # 路径的第一个节点必须与当前节点匹配
        if path[0] != current_node:
            return False

        # 路径只剩下当前节点，表示路径存在
        if len(path) == 1:
            return True

        # 检查路径的下一个节点是否是当前节点的邻居
        next_node = path[1]
        if next_node not in self.find_neighbor(current_node):
            return False

        # 递归检查剩余路径
        return self.path_exists_in_tree(next_node, path[1:])

    def path_exists(self, path):
        """检查路径是否存在（通用图结构）"""
        # 如果是树结构且有根节点
        #print("path_exists, path:", path)
        if self.root is not None:
            return self.path_exists_in_tree(self.root, path)

        # 通用图结构：检查路径中每对相邻节点是否相连
        for i in range(len(path) - 1):
            if path[i + 1] not in self.find_neighbor(path[i]):
                return False
        return True

    def get_leaves(self):
        """获取所有叶子节点"""
        leaves = []
        for node, neighbors in self.adjList.items():
            # 根节点不可能是叶子节点
            if self.is_leaf(node):
                leaves.append(node)
        return leaves

    def is_leaf(self, node):
        # 根节点不可能是叶子节点
        if node == self.root:
            return False
        # 非根节点且只有一个邻居的是叶子节点
        neighbors = self.adjList[node]
        if len(neighbors) <= 1:
            return True
        return False

    def sort(self):
        for node in self.adjList:
            neighbors = self.adjList[node]
            neighbors_sorted = self.get_sorted_nodes(neighbors)
            self.adjList[node] = neighbors_sorted

    def get_sorted_nodes(self, nodes):
        """
        获取按度数排序的节点列表

        排序规则:
        1. 首先按度数降序排序（度数高的在前）
        2. 度数相同时，按节点名称升序排序（字典序）

        :param nodes: 输入节点列表
        :return: 排序后的节点列表
        """
        node_degrees = {}

        # 计算每个节点的度数

        for node in nodes:
            # 如果节点在邻接表中，度数为邻居数量；否则度数为0
            neighbors = self.adjList.get(node)
            if neighbors is not None:
                node_degrees[node] = len(neighbors)
            else:
                node_degrees[node] = 0

        # 排序：先按度数降序，再按节点名称升序
        sorted_nodes = sorted(
            nodes,  # 对输入节点排序（保留原始节点顺序）
            key=lambda node: (
                -node_degrees[node],  # 度数降序（使用负号）
                node  # 名称升序
            )
        )

        # 调试输出（可选）
        # print("Sorted nodes:", sorted_nodes)
        # for node in sorted_nodes:
        #     print(f"Node: {node}, Degree: {node_degrees[node]}")

        return sorted_nodes

    def find_child_unconnected(self, node, connected):
        """查找节点的未连接子节点"""
        unconnected = 0
        unconnected_list = []

        with self.lock:
            if node in self.adjList:
                for neighbor in self.adjList[node]:
                    if neighbor not in connected:
                        unconnected += 1
                        unconnected_list.append(neighbor)

        return unconnected, unconnected_list

    def find_max_connected_neighbor(self, node, connected):
        """查找具有最多已连接邻居的邻居节点"""
        max_connected = -1
        max_neighbor = None

        with self.lock:
            if node in self.adjList:
                for neighbor in self.adjList[node]:
                    connected_count = 0
                    if neighbor in self.adjList:
                        for nn in self.adjList[neighbor]:
                            if nn in connected:
                                connected_count += 1

                    if connected_count > max_connected:
                        max_connected = connected_count
                        max_neighbor = neighbor

        return max_connected, max_neighbor

    def get_subgraph_within_hops(self, start_node, max_hops):
        """
        获取从起始节点在指定跳数范围内的子图
        :param start_node: 起始节点
        :param max_hops: 最大跳数
        :return: 子图对象
        """
        # 创建新的子图
        subgraph = Graph(self.env)

        # 使用队列进行BFS遍历
        queue = deque()
        queue.append((start_node, 0))  # (当前节点, 当前跳数)

        while queue:
            current_node, hops = queue.popleft()

            # 如果达到最大跳数，停止探索
            if hops >= max_hops:
                continue

            # 获取当前节点的邻居
            neighbors = self.find_neighbor(current_node)

            for neighbor in neighbors:
                if not subgraph.path_exists([current_node, neighbor]):
                    subgraph.add_edge(current_node, neighbor)
                    # 更新访问状态并加入队列
                    queue.append((neighbor, hops + 1))

        return subgraph

    def get_mlst(self, root):
        """构建最大叶子生成树（MLST）算法 - 线程安全版"""
        # 对邻接表进行排序以确保确定性
        self.sort()
        # 创建新的生成树
        mlstree = Graph(self.env)
        mlstree.root = root
        connected = {root: True}  # 已连接节点集合

        # 添加根节点的直接邻居
        with self.lock:
            if root in self.adjList:
                for neighbor in self.adjList[root]:
                    mlstree.add_edge(root, neighbor)
                    connected[neighbor] = True

        # 如果所有节点都已连接，直接返回
        if len(connected) == len(self.adjList):
            leaves = mlstree.get_leaves()
            return mlstree, leaves

        # 在二跳邻居中选择最佳节点
        max_unconnected = -1
        node_selected = None
        parent = None
        node_list = []

        # 遍历根节点的邻居节点
        with self.lock:
            if root in self.adjList:
                for node in self.adjList[root]:
                    # 遍历每个邻居的邻居
                        for nn in self.adjList[node]:
                            if nn not in connected:
                                unconnected, unconnected_list = self.find_child_unconnected(nn, connected)
                                if unconnected > max_unconnected:
                                    max_unconnected = unconnected
                                    node_selected = nn
                                    parent = node
                                    node_list = unconnected_list

        # 连接节点
        if parent:
            mlstree.add_edge(root, parent)
            connected[parent] = True

        if node_selected:
            mlstree.add_edge(parent, node_selected)
            connected[node_selected] = True
            for l in node_list:
                mlstree.add_edge(node_selected, l)
                connected[l] = True

        # 处理其他未连接的节点
        all_nodes = self.get_nodes()
        sorted_nodes = self.get_sorted_nodes(all_nodes)

        for node in sorted_nodes:
            if node not in connected:
                _, max_neighbor = self.find_max_connected_neighbor(node, connected)
                if max_neighbor:
                    mlstree.add_edge(max_neighbor, node)
                    connected[node] = True

        # 获取叶子节点
        leaves = mlstree.get_leaves()
        return mlstree, leaves

    def display(self):
        """显示图结构"""
        print("Current Network Topology:")
        for node, neighbors in self.adjList.items():
            print(f"{node}: {neighbors}")

