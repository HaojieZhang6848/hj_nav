import rclpy
from rclpy.node import Node
from argparse import ArgumentParser
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from multiprocessing.shared_memory import SharedMemory
import atexit
import logging
import os
from apriltag_msgs.msg import AprilTagDetectionArray

# 设置日志级别和格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

parser = ArgumentParser()
parser.add_argument('--debug', action='store_true')
args, unknown = parser.parse_known_args()

IS_DEUBG = args.debug

# 防止下面创建共享内存时出现FileExistsError
if os.path.exists('/dev/shm/red_detected'):
    os.remove('/dev/shm/red_detected')

# 创建共享内存，用来在red_detector和red_obj_server之间传递近5次检测是否有红色物体的结果
sz = 5
shm = SharedMemory(name='red_detected', create=True, size=sz)
def close_shm(shmm):
    if shmm and shmm._name:
        shmm.close()
        shmm.unlink()
atexit.register(close_shm, shm)

def update_red_detected(red_detected: bool):
    logging.debug(f'Update red_detected: {red_detected}')
    shm.buf[1:] = shm.buf[:-1]
    shm.buf[0] = int(red_detected)
    return
    
class RedDetectorNode(Node):

    def __init__(self):
        super().__init__('red_detector_node')
        self.get_logger().info('Red Detector Node started')
        
        # 创建april tag检测结果的订阅回调
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections', self.apriltag_callback, 10)
        
        # 创建tf广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def apriltag_callback(self, msg: AprilTagDetectionArray):
        detections = msg.detections
        if not detections:
            self.get_logger().debug('No AprilTag detected')
            update_red_detected(False)
            return
        update_red_detected(True)
        mark = detections[0]
        pose = mark.pose.pose.pose.position
        orientation = mark.pose.pose.pose.orientation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'red_object'
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = pose.z
        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w
        self.tf_broadcaster.sendTransform(t)        


def main():
    rclpy.init()
    node = RedDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
