from map_generation import Map
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np

class Path_Planner(Map):
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
    def A_start(self):
        self.g[self.start] = 0.0
        self.PARENT[self.start] = self.start
        heapq.heappush(self.OPEN, (0.0, self.start))
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
                    self.PARENT[neighbor] = current
    
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
    
if __name__ == '__main__':
    path_planner = Path_Planner()
    path_planner.set_start_end()
    path_planner.A_start()
    path_planner.draw_path()