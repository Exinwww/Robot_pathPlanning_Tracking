# Robot_pathPlanning_Tracking
一个栅格地图中的路径规划与轨迹跟踪实现

## 运行
* 使用默认参数运行 python main.py 
通过指定参数，调用不同算法，比如：
- 指定使用rrt进行轨迹规划，python main.py --planner rrt
- 指定使用pid进行轨迹跟踪，python main.py --traj pid

### 指定运行参数
* --map 指定地图，应该是文件夹map_image下的地图图片
* --planner [rrt 或者 astar] 指定路径规划算法
* --traj [pid 或 pure] 指定轨迹跟踪算法
* --kv --kp --ki --kd 指定pid参数
* --look_ahead_distance 指定pure算法参数
* --dt 指定路径平滑的时间间隔
* --b 指定差速robot的直径

## 文件说明
* Robot.py robot的实现，包括pid和pure算法的实现
* path_plan.py  路径规划算法
* map_generation.py 地图的生成
* path_visualization.py 地图与路径的可视化
* main.py 地图定义、轨迹规划、轨迹跟踪的完整流程

[注：通过planner.set_SG()函数可以在程序运行前即指定起点和终点，便于重复实现，见main.py的138行]