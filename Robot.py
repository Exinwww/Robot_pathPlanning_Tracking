"""定义机器人"""
import numpy as np
class ROBOT:
    def __init__(self, b=0.5, dt=0.1, kv=1.0, kp=1.0, ki=0.1, kd=0.01):
        """初始化机器人"""
        self.set_robot_param(b, dt)
        self.set_pid_param(kv, kp, ki, kd)
        self.set_state()

        self.integral_error = 0.0
        self.previous_error = 0.0
    def set_robot_param(self, b=0.5, dt=0.1):
        """设置机器人参数"""
        self.b = b
        self.dt = dt
    def set_state(self, x=0.0, y=0.0, theta=0.0):
        """设置机器人状态"""
        self.x = x
        self.y = y
        self.theta = theta

        self.x_traj = [self.x]
        self.y_traj = [self.y]
    def set_pid_param(self, kv=1.0, kp=1.0, ki=0.1, kd=0.01):
        """设置PID参数"""
        self.kv = kv
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def calculate_control(self, path):
        """计算控制量"""
        closest_point = min(path, key=lambda p:np.hypot(p[0]-self.x, p[1]-self.y))
        x_d, y_d = closest_point
        # 计算误差
        e_y = np.hypot(x_d - self.x, y_d - self.y)
        e_theta = np.arctan2(y_d - self.y, x_d - self.x) - self.theta

        # PID控制
        self.integral_error += e_theta * self.dt
        derivative_error = (e_theta - self.previous_error) / self.dt
        self.previous_error = e_theta

        v = self.kv * e_y
        omega = self.kp * e_theta + self.ki * self.integral_error + self.kd * derivative_error

        # 计算左右轮速度
        left = v - self.b * omega / 2
        right = v + self.b * omega / 2

        return left, right
    def PID_control(self, path):
        """
        PID控制算法
        """
        for i in range(len(path)):
            left, right = self.calculate_control(path[i:])
            # 更新机器人状态
            v = (left + right) / 2
            omega = (right - left) / self.b

            self.x += v * np.cos(self.theta) * self.dt
            self.y += v * np.sin(self.theta) * self.dt
            self.theta += omega * self.dt
            
            self.x_traj.append(self.x)
            self.y_traj.append(self.y)
        # 与终点存在一定距离时，继续控制
        self.end_control(path[-1])
    def end_control(self, goal, threshold=0.05):
        """
        控制机器人到达终点
        :param goal: 终点坐标
        :param threshold: 到达终点的阈值
        """
        while np.hypot(goal[0]-self.x, goal[1]-self.y) > threshold:
            left, right = self.calculate_control([goal])
            v = (left + right) / 2
            omega = (right - left) / self.b
            self.x += v * np.cos(self.theta) * self.dt
            self.y += v * np.sin(self.theta) * self.dt
            self.theta += omega * self.dt

            self.x_traj.append(self.x)
            self.y_traj.append(self.y)
    def get_traj(self):
        return self.x_traj, self.y_traj