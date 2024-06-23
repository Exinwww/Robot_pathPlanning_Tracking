"""
地图生成
"""
import numpy as np
import matplotlib.pyplot as plt
import cv2

class Map:
    def __init__(self, image_path='./map_image/map.png', threshold=128):
        """
        读入一个灰度图（或者彩色图），将其转换为一个地图
        """
        image = cv2.imread(image_path)
        self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, self.map = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)

        self.height, self.width = self.map.shape
        self.start = None
        self.goal = None
    def show_map_shape(self):
        print(f'height: {self.height}, width: {self.width}')

    def get_Start(self):
        """
        获取起点
        """
        print('Please click the start point on the map')
        def onclick(event):
            y,x = int(event.xdata), int(event.ydata)
            print(f"Clicked at: ({y}, {x})")
            self.start = (y, x)
            fig.canvas.mpl_disconnect(cid)
            plt.close()

        fig, ax = plt.subplots()
        ax.imshow(self.image)
        cid = fig.canvas.mpl_connect('button_press_event',onclick)
        plt.title('Getting Start Point')
        plt.show()

    def get_End(self):
        """
        获取终点
        """
        print('Please click the end point on the map')
        def onclick(event):
            y, x = int(event.xdata), int(event.ydata)
            print(f"Clicked at: ({y}, {x})")
            self.goal = (y, x)
            fig.canvas.mpl_disconnect(cid)
            plt.close()

        fig, ax = plt.subplots()
        ax.imshow(self.image)
        cid = fig.canvas.mpl_connect('button_press_event',onclick)
        plt.title('Getting End Point')
        plt.show()

    def set_start_end(self):
        self.get_Start()
        # 检查起点是否在地图内
        self.get_End()
        # 检查终点是否在地图内
    
    def showMap_with_Strat_End(self):
        plt.imshow(self.image)
        plt.scatter(self.start[0], self.start[1], c='r', s=10)
        plt.scatter(self.goal[0], self.goal[1], c='g', s=10)
        plt.axis('off')
        plt.show()
    # 计算曼哈顿距离
    def manhattan_distance(self, point1, point2):
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])
    
    def euler_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def chebyshev_distance(self, point1, point2):
        return max(abs(point1[0] - point2[0]), abs(point1[1] - point2[1]))
    # 得到周围的八个点
    def success(self, current_point):
        # 8个方向
        # directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        directions = [(1, 1), (-1, 1), (1, -1), (-1, -1), (1, 0), (0, 1), (-1, 0), (0, -1)]
        for dx, dy in directions:
            x, y = current_point
            x += dx
            y += dy
            if x < 0 or y < 0 or x >= self.width or y >= self.height:
                continue
            if self.map[y, x] == 0:
                continue
            yield x, y
    def is_Block(self, x:int, y:int):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        x = int(x)
        y = int(y)
        return self.map[y, x] == 0
    def set_SG(self, start, goal):
        self.start = start
        self.goal = goal

if __name__ == '__main__':
    map = Map()
    map.show_map_shape()
    map.set_start_end()
    map.showMap_with_Strat_End()