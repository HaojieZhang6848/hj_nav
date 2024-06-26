# dockerfile 更新日志

- 0.0.2：支持使用`INITIAL_POSE_X`、`INITIAL_POSE_Y`、`INITIAL_POSE_YAW`环境变量设置导航时小车的初始位置和朝向
- 0.0.3: 将limo和yb的total_launch都拆成nav_launch和red_launch，以允许分别启动红色检测程序和导航程序；red_detector和red_obj_server间增加了一块共享内存，传递是否检测到红色物体
- 0.0.4: 1.limo和x3统一导航参数 2.red_obj_detector在检测目标中点的深度时，如果中点的深度为0，会尝试中点周围的点的深度 3.更新地图
- 0.0.5: 1.导航参数xy_goal_tolerance设置为0.02, yaw_goal_tolerance设置为0.05 2.修复当检测到红色物体时，red_detector会在共享内存中写入多次的bug
- 0.0.6: 1.`loc_http_server`, `nav_http_server`, `red_obj_server`支持CORS 2.导航参数xy_goal_tolerance设置为0.1, yaw_goal_tolerance设置为0.1 3. 将启动相机的逻辑从`red_launch`中抽出来，成为`cam_launch`，并在`cam_launch`中启动web_video_server
- 0.0.7: `loc_http_server`, `nav_http_server`, `red_obj_server`支持`/health`接口
- 0.0.8: 在`red_detector`启动5秒后才启动`red_obj_server`，防止出现race condition；加入vim
- 0.0.9: 在`nav_http_server`中添加了`/initial_pose`接口，用于设置小车的初始位置和朝向(amcl)
- 0.0.9-7: 添加中途取消导航的功能