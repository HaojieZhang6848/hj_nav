FROM huajuan6848/limo_ros2:0.0.3

WORKDIR /root/agx_ws

RUN mkdir -p src/hj_nav

# 复制源代码
COPY . src/hj_nav

# 湾谷地图
COPY docker/map/wangu/* src/limo_ros2/limo_bringup/maps

# 加入vim
RUN apt-get update && apt-get install -y vim

# 安装依赖
RUN bash -c "pip3 install -r src/hj_nav/nav_http_server/requirements.txt"
RUN bash -c "pip3 install -r src/hj_nav/red_detector/requirements.txt"
RUN bash -c "pip3 install -r src/hj_nav/hj_nav_launch/requirements.txt"
RUN bash -c "source /opt/ros/foxy/setup.bash && rosdep update --rosdistro foxy && rosdep install --from-paths src --ignore-src -r -y"

# 导航参数
COPY docker/limo/navigation2.yaml src/limo_ros2/limo_bringup/param/navigation2.yaml

# 编译
RUN bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && colcon build --symlink-install"