from Robot import ROBOT
from path_plan import Astar_Planner, RRT_Planner
from map_generation import Map
import matplotlib.pyplot as plt
from path_visualization import Visualizer
import numpy as np

if __name__ == '__main__':
    dt = 0.5 # 设置时间间隔
    # 路径规划
    # planner = Astar_Planner(image_path='./map_image/map2.png')
    # planner.set_start_end()
    # planner.A_start()
    # print(f'path length: {len(planner.backtrace())}')
    # path = planner.backtrace()
    # path = planner.path_subdivision(dt=dt)

    planner = RRT_Planner()
    planner.set_start_end()
    planner.RRT()
    path = planner.extract_path()
    print(f'path_rrt length: {len(path)}')
    path = planner.path_subdivision(dt=dt)

    # 路径跟踪
    robot = ROBOT(dt=dt)
    robot.set_state(*planner.start, 0.0)
    robot.set_pid_param(kp=2.0, ki=0.1, kd=0.1) # kp若为1.0，会出现问题; best:2.0,0.5,0.1
    robot.PID_control(path)

    x_traj, y_traj = robot.get_traj()
    print(f'x_traj: {len(x_traj)}, y_traj: {len(y_traj)}')
    print(f'Path End: {path[-1]}')
    print(f'Trajectory End: {x_traj[-1]}, {y_traj[-1]}')
    
    # 绘制地图
    # plt.imshow(Astar_Planner.image)
    # # 绘制规划路径
    # path_x = [point[0] for point in path]
    # path_y = [point[1] for point in path]
    # plt.plot(path_x, path_y, c='r')
    # plt.gca().axes.get_xaxis().set_ticks([])
    # plt.gca().axes.get_yaxis().set_ticks([])
    # # 绘制起终点
    # plt.scatter(Astar_Planner.start[0], Astar_Planner.start[1], c='black', s=10)
    # plt.scatter(Astar_Planner.goal[0], Astar_Planner.goal[1], c='green', s=10)
    # # 绘制追踪轨迹
    # plt.plot(robot.x_traj, robot.y_traj, c='b')
    # plt.title('Path Tracking')
    # plt.show()

    # 可视化
    visualizer = Visualizer(target_path=np.array(path), trac_path=np.array([x_traj, y_traj]).T, image=planner.image)
    visualizer.show()
