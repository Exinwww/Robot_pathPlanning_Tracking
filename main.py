from Robot import ROBOT
from path_plan import Astar_Planner, RRT_Planner
from path_visualization import Visualizer
import numpy as np
import argparse
from fastdtw import fastdtw

# 设置命令行参数解析
parser = argparse.ArgumentParser(description='Path Planning and Tracking')
# 地图文件路径
parser.add_argument('--map', 
                    type=str, 
                    default='./map_image/map2.png', 
                    help='Path to the map image')
# 对已规划的路径作平滑处理的时间间隔
parser.add_argument('--dt',
                    type=float,
                    default=0.5,
                    help='Time interval to subdivide the path')
# 选择路径规划器 A*或RRT
parser.add_argument('--planner',
                    type=str,
                    default='astar',
                    help='Path planner to use: astar or rrt')
# 选择路径跟踪器 PID或Pure Pursuit
parser.add_argument('--traj',
                    type=str,
                    default='pure',
                    help='Trajectory tracking algorithm to use: pid or pure')
# 设置PID的参数
parser.add_argument('--kv',
                    type=float,
                    default=1.0,
                    help='Velocity gain')
parser.add_argument('--kp',
                    type=float,
                    default=2.0,
                    help='Proportional gain')
parser.add_argument('--ki',
                    type=float,
                    default=0.1,
                    help='Integral gain')
parser.add_argument('--kd',
                    type=float,
                    default=0.1,
                    help='Derivative gain')
# 设置Pure Pursuit的参数
parser.add_argument('--look_ahead_distance',
                    type=float,
                    default=1.0,
                    help='Look ahead distance for pure pursuit')
# 设置机器人的参数
parser.add_argument('--b',
                    type=float,
                    default=0.5,
                    help='Wheel base of the robot')
args = parser.parse_args()

def test():
    map_path = './map_image/map2.png'
    dt = 0.5 # 设置时间间隔
    # 路径规划
    # planner = Astar_Planner(image_path='./map_image/map2.png')
    # planner.set_start_end()
    # planner.A_start()
    # print(f'path length: {len(planner.backtrace())}')
    # path = planner.backtrace()
    # path = planner.path_subdivision(dt=dt)

    planner = RRT_Planner(image_path=map_path)
    planner.set_start_end()
    planner.RRT()
    path = planner.backtrace()
    print(f'path_rrt length: {len(path)}')
    path = planner.path_subdivision(dt=dt)

    # 路径跟踪
    robot = ROBOT(dt=dt)
    robot.set_state(*planner.start, 0.0)
    robot.set_pid_param(kp=2.0, ki=0.1, kd=0.1) # kp若为1.0，会出现问题; best:2.0,0.5,0.1
    # robot.PID_control(path)
    robot.pure_pursuit_control(path)

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

# 计算轨迹跟踪的误差
def evalution(target_path, traj_path):
    # 计算DTW距离矩阵
    distance, path = fastdtw(target_path, traj_path)
    # 计算误差
    errors = []
    for (i, j) in path:
        point_target = target_path[i]
        point_actual = traj_path[j]
        error = np.linalg.norm(point_target - point_actual)
        errors.append(error)
    mean_error = np.mean(errors)
    print("\nMean DTW error:", mean_error)
    return mean_error

if __name__ == '__main__':
    map_path = args.map
    # 设置路径规划器
    planner = None
    print(f'### Planner: {args.planner}')
    if args.planner == 'astar':
        planner = Astar_Planner(image_path=map_path)
    elif args.planner == 'rrt':
        planner = RRT_Planner(image_path=map_path)
    else:
        raise ValueError('Planner not supported')
    planner.set_start_end() # 设置起终点
    # planner.set_SG((3, 4), (29, 47)) # 程序运行前设置起终点 for map
    # planner.set_SG((3, 54), (95, 28)) # 程序运行前设置起终点 for map2
    planner.plan() # 规划路径
    path = planner.backtrace() # 获取规划路径
    path = planner.path_subdivision(dt=args.dt) # 对规划路径进行平滑处理
    print('Get the path')

    # 设置轨迹跟踪器
    print(f'### Trajectory tracking: {args.traj}')
    robot = robot = ROBOT(b=args.b, dt=args.dt)
    robot.set_state(*planner.start, 0.0)
    if args.traj == 'pid':
        robot.set_pid_param(kv=args.kv, kp=args.kp, ki=args.ki, kd=args.kd)
        robot.PID_control(path)
    elif args.traj == 'pure':
        robot.set_pure_param(args.look_ahead_distance)
        robot.pure_pursuit_control(path)
    else:
        raise ValueError('Trajectory tracking algorithm not supported')
    x_traj, y_traj = robot.get_traj()
    print('Get the trajectory')

    # 计算轨迹跟踪的误差
    traj_path = [ [x_traj[i], y_traj[i]] for i in range(len(x_traj))]
    evalution(np.array(path), np.array(traj_path))
    # 输出时间、访问节点数统计量
    print(f'Planning time: {planner.plan_time:.2f}s')
    print(f'Visited nodes: {planner.node_counter}')

    # 可视化
    visualizer = Visualizer(target_path=np.array(path), trac_path=np.array(traj_path), image=planner.image)
    visualizer.show()
    
    # 用于在使用RRT规划器时，对PID和Pure Pursuit进行比较（因为RRT具有随机性，所以每次规划结果不同）
    # if args.planner == 'rrt':
    #     new_robot = None
    #     if args.traj != 'pid':
    #         new_robot = ROBOT(b=args.b, dt=args.dt)
    #         new_robot.set_state(*planner.start, 0.0)
    #         new_robot.set_pid_param(kv=args.kv, kp=args.kp, ki=args.ki, kd=args.kd)
    #         new_robot.PID_control(path)
    #     else:
    #         new_robot = ROBOT(b=args.b, dt=args.dt)
    #         new_robot.set_state(*planner.start, 0.0)
    #         new_robot.set_pure_param(args.look_ahead_distance)
    #         new_robot.pure_pursuit_control(path)
        
    #     x_traj, y_traj = new_robot.get_traj()

    #     # 计算轨迹跟踪的误差
    #     traj_path = [ [x_traj[i], y_traj[i]] for i in range(len(x_traj))]
    #     evalution(np.array(path), np.array(traj_path))
    #     # 输出时间、访问节点数统计量
    #     print(f'Planning time: {planner.plan_time:.2f}s')
    #     print(f'Visited nodes: {planner.node_counter}')

    #     # 可视化
    #     visualizer = Visualizer(target_path=np.array(path), trac_path=np.array(traj_path), image=planner.image)
    #     visualizer.show()
