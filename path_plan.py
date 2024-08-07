from map_generation import Map
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
import time

class Astar_Planner(Map):
    def __init__(self, image_path='./map_image/map.png', threshold=128):
        super().__init__(image_path, threshold)
        self.OPEN=[]
        self.CLOSED=[]
        self.g = {}
        self.PARENT = {}

    def heuristic(self, a):
        """
        计算启发式函数
        """
        return self.euler_distance(a, self.goal)
    def move_cost(self, a, b):
        """
        计算移动代价
        """
        return self.manhattan_distance(a, b)
    def plan(self):
        # 获取当前时间
        start_time = time.time()
        # 初始化
        self.g[self.start] = 0.0
        self.PARENT[self.start] = self.start
        heapq.heappush(self.OPEN, (0.0, self.start))
        self.node_counter = 1 # 访问节点计数器
        # A*算法
        cs = ['-', '\\', '|', '/']
        counter = 0
        while len(self.OPEN) > 0:
            print(f'In Astart {cs[counter%4]}', end='\r')
            counter += 1

            current = heapq.heappop(self.OPEN)[1]
            self.CLOSED.append(current)
            if current == self.goal:
                print('Find the goal')
                break
            for neighbor in self.success(current):
                new_g = self.g[current] + self.move_cost(current, neighbor)
                if neighbor in self.CLOSED:
                    continue
                if neighbor not in self.g or self.g[neighbor] > new_g:
                    self.g[neighbor] = new_g
                    f = new_g + self.heuristic(neighbor)
                    heapq.heappush(self.OPEN, (f, neighbor))
                    self.node_counter += 1 # 记录访问节点数
                    self.PARENT[neighbor] = current
        
        # 获取结束时间
        end_time = time.time()
        self.plan_time = end_time - start_time
    
    # 回溯路径
    def backtrace(self):
        path = [self.goal]
        while path[-1] != self.start:
            path.append(self.PARENT[path[-1]])
        return path[::-1]

    # 路径绘制
    def draw_path(self):
        path = self.backtrace()
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.imshow(self.image)
        plt.scatter(self.start[0], self.start[1], c='black', s=10)
        plt.scatter(self.goal[0], self.goal[1], c='green', s=10)
        plt.plot(path_x, path_y, c='r')
        plt.gca().axes.get_xaxis().set_ticks([])
        plt.gca().axes.get_yaxis().set_ticks([])
        plt.title('Path Planning')
        plt.show()

    # 按照给定的时间间隔dt对路径进行插值
    def path_subdivision(self, dt=0.1):
        path = self.backtrace()
        new_path = []
        for i in range(len(path)-1):
            distance = self.euler_distance(path[i], path[i+1])
            num = math.ceil(distance / dt)
            # print(f'distance: {distance}, num: {num}')
            x = np.linspace(path[i][0], path[i+1][0], num)
            y = np.linspace(path[i][1], path[i+1][1], num)
            new_path.extend(list(zip(x, y)))
        new_path.append(self.goal)
        return new_path
    
class Node:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
        self.parent = None

class RRT_Planner(Map):
    def __init__(self, image_path='./map_image/map.png', threshold=128, step_size=2.0):
        super().__init__(image_path, threshold)
        self.tree = []
        self.goal_node = None
        self.step_size = step_size
    def get_random_node(self):
        """地图范围内随机采样一个点"""
        while True:
            x = np.random.randint(0, self.width)
            y = np.random.randint(0, self.height)
            if not self.is_Block(x, y):
                return Node(x, y)
    def nearest_node(self, random_node):
        """找到树中距离随机最近的节点"""
        return min(self.tree, key=lambda node:self.euler_distance((node.x, node.y), (random_node.x, random_node.y)))
    def steer(self, from_node, to_node):
        """从一个结点项另一个结点扩展"""
        angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = np.ceil(from_node.x + self.step_size * np.cos(angle))
        new_y = np.ceil(from_node.y + self.step_size * np.sin(angle))
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node
    def is_goal(self, node):
        """判断是否到达目标点"""
        return self.euler_distance((node.x, node.y), self.goal) < 2.0
    def line_is_Block(self, from_node, to_node):
        """判断两个点之间是否有障碍物"""
        distance = self.euler_distance((from_node.x, from_node.y), (to_node.x, to_node.y))
        num = math.ceil(distance) * 10
        x = np.linspace(from_node.x, to_node.x, num)
        y = np.linspace(from_node.y, to_node.y, num)
        for i in range(num):
            if self.is_Block(x[i], y[i]):
                return True
        return False
    def backtrace(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
    def plan(self, max_iter=500000):
        """RRT的实现"""
        # 获取当前时间
        start_time = time.time()
        self.node_counter = 0 # 访问节点计数器
        # 初始化
        start_node = Node(*self.start)
        goal_node = Node(*self.goal)
        self.tree.append(start_node)
        cs = ['-', '\\', '|', '/']
            
        for _ in range(max_iter):
            random_node = self.get_random_node()
            nearest = self.nearest_node(random_node)
            new_node = self.steer(nearest, random_node)
            self.node_counter += 1 # 记录访问节点数
            print(f'In RRT {cs[self.node_counter%4]}', end='\r') # 进度条
            if not self.is_Block(new_node.x, new_node.y) and not self.line_is_Block(nearest, new_node):
                self.tree.append(new_node)
                if self.is_goal(new_node):
                    goal_node.parent = new_node
                    self.goal_node = goal_node
                    print('Find the goal')
                    break

        # 获取结束时间
        end_time = time.time()
        self.plan_time = end_time - start_time
    def path_subdivision(self, dt=0.1):
        path = self.backtrace()
        new_path = []
        for i in range(len(path)-1):
            distance = self.euler_distance(path[i], path[i+1])
            num = math.ceil(distance / dt)
            x = np.linspace(path[i][0], path[i+1][0], num)
            y = np.linspace(path[i][1], path[i+1][1], num)
            new_path.extend(list(zip(x, y)))
        new_path.append(self.goal)
        return new_path
    def draw_path(self):
        """绘制路径"""
        path = self.backtrace()
        print(f'path length: {len(path)}')
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.imshow(self.image)
        plt.scatter(self.start[0], self.start[1], c='black', s=10)
        plt.scatter(self.goal[0], self.goal[1], c='green', s=10)
        plt.plot(path_x, path_y, c='r')
        plt.gca().axes.get_xaxis().set_ticks([])
        plt.gca().axes.get_yaxis().set_ticks([])
        plt.title('Path Planning')
        plt.show()
    
if __name__ == '__main__':
    # path_planner = Astar_Planner()
    # path_planner.set_start_end()
    # path_planner.A_start()
    # path_planner.draw_path()

    rrt_planner = RRT_Planner()
    rrt_planner.set_start_end()
    rrt_planner.plan()
    rrt_planner.draw_path()