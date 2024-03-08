import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from argparse import ArgumentParser
from sensor_msgs.msg import CameraInfo
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
try:
    from .utils import quaternion_to_eular, eular_to_quaternion  # for ros2 run
except:
    from utils import quaternion_to_eular, eular_to_quaternion  # for direct run
from multiprocessing.shared_memory import SharedMemory
import atexit
import logging

# 设置日志级别和格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

parser = ArgumentParser()
parser.add_argument('--debug', action='store_true')
args, unknown = parser.parse_known_args()

IS_DEUBG = args.debug

# 创建共享内存，用来在red_detector和red_obj_server之间传递近5次检测是否有红色物体的结果
sz = 5
shm = SharedMemory(name='red_detected', create=True, size=sz)
def close_shm(shmm):
    if shmm and shmm._name:
        shmm.close()
        shmm.unlink()
atexit.register(close_shm, shm)

def update_red_detected(red_detected: bool):
    logging.info(f'Update red_detected: {red_detected}')
    shm.buf[1:] = shm.buf[:-1]
    shm.buf[0] = int(red_detected)
    return
    
class RedDetectorNode(Node):

    def __init__(self):
        super().__init__('red_detector_node')
        self.get_logger().info('Red Detector Node started')
        self.color = None
        self.color_timestamp = 0
        self.depth = None
        self.depth_timestamp = 0
        self.k = None

        self.cv_bridge = CvBridge()

        # 创建颜色图像和深度图像的订阅回调
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # 创建定时器
        self.timer = self.create_timer(1, self.on_timer)

        # 创建tf广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def color_callback(self, msg):
        self.color = msg
        self.color_timestamp = int(time.time() * 1000)

    def depth_callback(self, msg):
        self.depth = msg
        self.depth_timestamp = int(time.time() * 1000)

    def camera_info_callback(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def check_color_depth(self):
        # 检测color和depth是否都有数据，且都在最近的1000ms内，并输出日志
        if self.color is not None and self.depth is not None:
            if abs(self.color_timestamp - self.depth_timestamp) < 2000:
                self.get_logger().info('Color and depth are both available')
                return True
            else:
                self.get_logger().warn('Color and depth are not synchronized')
        else:
            self.get_logger().warn('Color or depth is not available')

        return False

    def check_camera_k(self):
        if self.k is not None:
            self.get_logger().info('Camera info is available')
            return True
        else:
            self.get_logger().warn('Camera info is not available')
            return False

    def save_debug_image(self, color_image, depth_colormap):
        if not IS_DEUBG:
            return
        # 保存调试图像
        cv2.imwrite('color_image.jpg', color_image)
        cv2.imwrite('depth_colormap.jpg', depth_colormap)

    def get_red_contours(self, color_image):
        # 将图像转换为HSV格式
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # 设定红色的阈值
        lower_red_1 = np.array([0, 100, 100])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 100, 100])
        upper_red_2 = np.array([180, 255, 255])

        # 根据阈值构建掩模
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 执行形态学操作以去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 寻找红色物体的轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    def broadcast_tf(self, xyz):
        # 导入pi
        from math import pi

        # 计算红色目标在camera_color_frame中的坐标
        # 已知camera_color_frame -> camera_color_optical_frame的旋转是[-pi/2, pi/2, 0], 平移是[0, 0, 0]
        # 已知红色目标在camera_color_optical_frame中的坐标是xyz[0], xyz[1], xyz[2]
        t_hat = np.array([xyz[2], -xyz[0], -xyz[1]])
        
        op0 = np.array([-np.sqrt(t_hat[0]**2 + t_hat[1]**2), 0])
        
        # 红色目标的tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_frame'
        t.child_frame_id = 'red_object'
        t.transform.translation.x = t_hat[0]
        t.transform.translation.y = t_hat[1]
        t.transform.translation.z = t_hat[2]
        rpy = [0, 0, np.arctan2(t_hat[1], t_hat[0])]
        q = eular_to_quaternion(rpy[0], rpy[1], rpy[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 顺时针和逆时针旋转120度u的旋转矩阵
        theta1 = 2 * pi / 3
        theta2 = - 2 * pi / 3
        u1 = np.array([[np.cos(theta1), -np.sin(theta1)], [np.sin(theta1), np.cos(theta1)]])
        u2 = np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])

        op1 = np.dot(u1, op0)
        op2 = np.dot(u2, op0)

        q1 = eular_to_quaternion(0, 0, theta1)
        q2 = eular_to_quaternion(0, 0, theta2)

        # 另一个小车1的tf
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'red_object'
        t1.child_frame_id = 'another_car_1'
        t1.transform.translation.x = op1[0]
        t1.transform.translation.y = op1[1]
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]
        self.tf_broadcaster.sendTransform(t1)

        # 另一个小车2的tf
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'red_object'
        t2.child_frame_id = 'another_car_2'
        t2.transform.translation.x = op2[0]
        t2.transform.translation.y = op2[1]
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]
        self.tf_broadcaster.sendTransform(t2)

    def on_timer(self):
        if not self.check_color_depth():
            return update_red_detected(False)
        if not self.check_camera_k():
            return update_red_detected(False)

        # 将ROS消息转换为OpenCV格式
        color_image = self.cv_bridge.imgmsg_to_cv2(self.color, 'bgr8')
        depth_image = self.cv_bridge.imgmsg_to_cv2(self.depth, 'passthrough')

        # 深度图转伪彩色图
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # 获取红色轮廓
        red_contours = self.get_red_contours(color_image)

        if len(red_contours) == 0:
            self.get_logger().info('No red object detected')
            return update_red_detected(False)

        # 找到red_contours中最大的那个的外接矩形，计算其中点坐标，绘制矩形
        max_contour = max(red_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        cx, cy = x + w // 2, y + h // 2
        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

        # 保存调试图像
        self.save_debug_image(color_image, depth_colormap)
        
        # 获得目标中点或附近的深度值
        def get_depth(cv_x, cv_y):
            # 如果cv_x, cv_y的深度值为0，则在附近尝试找一个不为0的深度值
            if depth_image[cv_y, cv_x] != 0:
                return depth_image[cv_y, cv_x] / 1000.0
            else:
                for i in range(-3, 2):
                    for j in range(-3, 2):
                        # 判断是否越界
                        if cv_x + i < 0 or cv_x + i >= depth_image.shape[1] or cv_y + j < 0 or cv_y + j >= depth_image.shape[0]:
                            continue
                        if depth_image[cv_y + j, cv_x + i] != 0:
                            return depth_image[cv_y + j, cv_x + i] / 1000.0
            return 0
        depth = get_depth(cx, cy)
        if depth == 0:
            self.get_logger().warn('Depth at center is 0')
            return update_red_detected(False)
        
        self.get_logger().info(f'Depth at center: {depth} m')

        # 根据相机内参计算中点的3D坐标
        k_inv = np.linalg.inv(self.k)
        xyz = depth * np.dot(k_inv, np.array([cx, cy, 1]))
        self.get_logger().info(f'3D coordinate at center: {xyz}')

        # 发布中点的3D坐标到tf中（上面计算出来的xyz，相对于相机坐标系camera_color_optical_frame）
        try:
            self.broadcast_tf(xyz)
            update_red_detected(True)
        finally:
            update_red_detected(False)


def main():
    rclpy.init()
    node = RedDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
