""" 可视化"""
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self, target_path=None, trac_path=None, image =None) -> None:
        self.target_path = target_path
        self.trac_path = trac_path
        self.background = image
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 4))
        
        self.ax1.imshow(self.background)
        self.ax1.set_title('Planned Path')
        self.ax1.set_xlim(0, self.background.shape[1])
        self.ax1.set_ylim(0, self.background.shape[0])
        self.ax1.invert_yaxis()  # 反转y轴，使y轴从上到下递增         
        self.ax1.set_xticks([])
        self.ax1.set_yticks([])
        self.ax1.scatter(self.target_path[0][0], self.target_path[0][1], c='black', s=10)
        self.ax1.scatter(self.target_path[-1][0], self.target_path[-1][1], c='green', s=10)

        self.ax2.imshow(self.background)
        self.ax2.set_title('Tracking Path')
        self.ax2.set_xlim(0, self.background.shape[1])
        self.ax2.set_ylim(0, self.background.shape[0])
        self.ax2.invert_yaxis()  # 反转y轴，使y轴从上到下递增        
        self.ax2.set_xticks([])
        self.ax2.set_yticks([])
        self.ax2.scatter(self.trac_path[0][0], self.trac_path[0][1], c='black', s=10)
        self.ax2.scatter(self.trac_path[-1][0], self.trac_path[-1][1], c='green', s=10)
        self.line1, = self.ax1.plot([], [], 'r-')
        self.line2, = self.ax2.plot([], [], 'b-')
        self.line3, = self.ax2.plot([], [], 'r-')
        self.ani_left = FuncAnimation(self.fig, self.update, frames=len(self.target_path)+len(self.trac_path), init_func=self.init, blit=True, interval=50, repeat=False)
        
    def init(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        self.line3.set_data([], [])
        return self.line1, self.line3, self.line2

    def update(self, frame):
        if frame < len(self.target_path):
            x = [point[0] for point in self.target_path[:frame]]
            y = [point[1] for point in self.target_path[:frame]]
            self.line1.set_data(x, y)
            return self.line1,
        else:
            x2 = [point[0] for point in self.trac_path[:frame-len(self.target_path)]]
            y2 = [point[1] for point in self.trac_path[:frame-len(self.target_path)]]
            self.line2.set_data(x2, y2)
            x3 = [point[0] for point in self.target_path]
            y3 = [point[1] for point in self.target_path]
            self.line3.set_data(x3, y3)
            return self.line3, self.line2
    
    def show(self):
        plt.tight_layout()
        plt.show()
        