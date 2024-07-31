# 湾谷小车程序使用方法及接口说明

## 启动镜像

小车有`limo`和`x3`两种类型，分别对应不同的镜像。

`limo`的镜像为[huajuan6848/x3-54demo-ros-foxy](https://hub.docker.com/repository/docker/huajuan6848/limo-54demo-ros-foxy/general)，`x3`的镜像为[huajuan6848/x3-54demo-ros-foxy](https://hub.docker.com/repository/docker/huajuan6848/x3-54demo-ros-foxy/general)。

镜像会有时更新，注意使用最新的镜像。

### limo

在limo小车上可以使用以下命令启动镜像。如果是通过kubeedge或k3s在集群中启动，请自行将下面的docker命令改写成pod的yaml文件。

```bash
docker run \
        -d \
        --network=host \
        -e ROS_DOMAIN_ID=28 \ # ROS2的domain id，不同的小车使用不同的domain id，随便找一个未被占用的数字即可
        -v /dev:/dev \
        --privileged \
        --restart=always \
        --name=limo_dev \
        -e INITIAL_POSE_X=0.0 \ # 小车初始位置x坐标
        -e INITIAL_POSE_Y=0.0 \ # 小车初始位置y坐标
        -e INITIAL_POSE_YAW=0.0 \ # 小车初始朝向
        huajuan6848/limo-54demo-ros-foxy:0.0.5
```

limo小车镜像的默认entrypoint是一个无限sleep的程序。如果需要进入容器内部，可以使用以下命令：

```bash
docker exec -it limo_dev bash
```

### x3

在x3小车上可以使用以下命令启动镜像。如果是通过kubeedge或k3s在集群中启动，请自行将下面的docker命令改写成pod的yaml文件。

```bash
docker run -it \
    --net=host \
    --env="ROS_DOMAIN_ID=27" \ # ROS2的domain id，不同的小车使用不同的domain id，随便找一个未被占用的数字即可
    --privileged \
    -v /dev:/dev \
    --security-opt apparmor:unconfined \
    -e INITIAL_POSE_X=0.0 \ # 小车初始位置x坐标
    -e INITIAL_POSE_Y=0.0 \ # 小车初始位置y坐标
    -e INITIAL_POSE_YAW=0.0 \ # 小车初始朝向
    -e RPLIDAR_TYPE = <your_rplidar_type> \ # 你的激光雷达型号，jetson版小车是s2，树莓派版小车是a1
    huajuan6848/x3-54demo-ros-foxy:0.0.5 /bin/bash
```

x3小车镜像的默认entrypoint是什么我忘了，可能你需要手动设置一下，例如设置成和limo一样的无限sleep(`sleep infinity`)

## 启动功能

目前，小车的功能分为两部分：红色物体检测和导航。两部分功能是分别启动的。

### 启动导航

limo小车
```bash
ros2 launch hj_nav_launch limocar_nav_launch.py 
```
x3小车
```bash
ros2 launch hj_nav_launch ybcar_nav_launch.py
```

### 启动相机
limo小车
```bash
ros2 launch hj_nav_launch limocar_cam_launch.py
```
x3小车
```bash
ros2 launch hj_nav_launch ybcar_cam_launch.py
```

这会同时启动相机节点和`web_video_server`节点。 limo小车的相机型号是dabai_u3，x3小车的相机型号是astro_pro_plus。

可以在浏览器中查看小车的摄像头画面。用浏览器访问`http://<小车ip>:8080`即可。

### 启动红色物体检测

请先确认你已经完成了上面所说的“启动相机“的步骤。

limo小车
```bash
ros2 launch hj_red_launch limocar_red_launch.py
```

x3小车
```bash
ros2 launch hj_red_launch ybcar_red_launch.py
```

## 接口说明

### 红色物体检测

URL: `http://<小车ip>:5002/red_loc`

METHOD: GET

返回值：json格式，例如
```json
{
    "another_car_1": {
        "theta": 2.489903207615628,
        "x": 5.591663248985672,
        "y": -0.15129701940983808
    },
    "another_car_2": {
        "theta": -1.6988869250472636,
        "x": 5.040084792525069,
        "y": 1.1698138248626422
    },
    "red_object": {
        "theta": 0.39550127354399406,
        "x": 4.934500743700147,
        "y": 0.35003492304857814
    }
}
```

### 获得小车的位置和朝向

URL: `http://<小车ip>:5001/loc`

METHOD: GET

返回值：json格式，例如
```json
{
    "theta": 0.23205177501413307,
    "x": 4.747510171210837,
    "y": 0.27493159132896106
}
```

### 获得小车是否正在移动

URL: `http://<小车ip>:5000/is_moving`

METHOD: GET

返回值：json格式，例如
```json
{
    "is_moving": false
}
```

### 导航小车到指定位置和朝向

URL: `http://<小车ip>:5000/nav`

METHOD: POST

POST数据：json格式，例如
```json
{
    "x": 4.0,
    "y": 0.0,
    "theta": 0.0
}
```

返回值：json格式，例如
```json
{
    "status": "success",
    "message": "navigation goal received"
}
```

### 导航小车到指定位置和朝向（同步）

URL: `http://<小车ip>:5000/nav_sync`

其余同上。同步的意思是，这个请求会一直等到小车到达目标位置才返回。

### 错误处理

如果以上的请求返回的状态码不是200，说明请求出现了错误。错误的信息会在返回的json中给出。

错误的信息格式如下：
```json
{
    "path": "请求的路径",
    "method": "请求的方法",
    "timestamp": "时间戳，是1970年1月1日到现在的秒数",
    "code": "错误的状态码",
    "message": "错误的信息"
}
```