from flask import Flask, request, jsonify, make_response
from waitress import serve
from argparse import ArgumentParser
from rclpy.action import ActionClient
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from datetime import datetime
try:
    from .utils import eular_to_quaternion  # for ros2 run
    from .models import NavRequest, InitialPoseRequest
except:
    from utils import eular_to_quaternion  # for python3 nav_http_server.py
    from models import NavRequest, InitialPoseRequest
import logging
from threading import Thread
from flask_cors import CORS
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
import tf2_ros
try:
    from .utils import quaternion_to_eular  # for ros2 run
except:
    from utils import quaternion_to_eular  # for python3 nav_http_server.py

# 设置日志级别和格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 解析--map-frame和--navigate-to-pose-action两个参数
parser = ArgumentParser()
parser.add_argument('--map-frame', default='map')
parser.add_argument('--navigate-to-pose-action', default='/navigate_to_pose')
parser.add_argument('--robot-frame', default='base_link')
args, unknown = parser.parse_known_args()

MAP_FRAME = args.map_frame
ROBOT_FRAME = args.robot_frame
NAVIGATE_TO_POSE_ACTION = args.navigate_to_pose_action

DIST_THRESHOLD = 0.01
ROT_THRESHOLD = 0.01

class SendNavGoalNode(rclpy.node.Node):
    def __init__(self):
        ts = datetime.now().timestamp()
        super().__init__('nav_http_server' + str(ts).replace('.', ''))
        self.action_client = ActionClient(self, NavigateToPose, NAVIGATE_TO_POSE_ACTION)
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available after waiting')
            exit(1)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_handles = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
    
    def send_initial_pose(self, x, y, theta):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = MAP_FRAME
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = 0.0
       
        q = eular_to_quaternion(0, 0, theta)
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]
        
        self.initial_pose_publisher.publish(pose_msg)
        
    def __send_nav_goal_base(self, x, y, theta):
        q = eular_to_quaternion(0, 0, theta)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = MAP_FRAME
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        goal_handle_future = self.action_client.send_goal_async(goal_msg)
        import time
        time.sleep(0.1)
        goal_handle = goal_handle_future.result()
        
        return goal_handle
        
    def send_nav_goal(self, x, y, theta):
        goal_handle = self.__send_nav_goal_base(x, y, theta)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False, 'goal rejected'
        self.get_logger().info('Move started')
        
        self.goal_handles.append(goal_handle)
        
        return True, 'goal accepted'
    
    def send_nav_goal_sync(self, x, y, theta):
        goal_handle = self.__send_nav_goal_base(x, y, theta)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False, 'goal rejected'
        self.get_logger().info('Move started')
        self.goal_handles.append(goal_handle)
        
        # wait for the goal handle to finish
        not_move_statuses = [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]
        import time
        while goal_handle.status not in not_move_statuses:
            time.sleep(0.1)

        result_status = goal_handle.status
        result_status_str = ''
        if result_status == GoalStatus.STATUS_SUCCEEDED:
            result_status_str = 'SUCCEEDED'
        elif result_status == GoalStatus.STATUS_CANCELED:
            result_status_str = 'CANCELED'
        elif result_status == GoalStatus.STATUS_ABORTED:
            result_status_str = 'ABORTED'
        
        self.get_logger().info(f'Move finished with status: {result_status_str}')
        return True, f'move finished with status: {result_status_str}'
    
    def is_moving(self):
        if len(self.goal_handles) == 0:
            return False
        gh = self.goal_handles[-1]
        not_is_moving_statuses = [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]
        return gh.status not in not_is_moving_statuses
    
    def cancel_nav(self):
        if len(self.goal_handles) == 0:
            return False, 'no goal to cancel'
        gh = self.goal_handles[-1]
        gh.cancel_goal()
        return True, 'goal canceled'
    
    def robot_loc(self):
        # 获得ROBOT_FRAME相对于MAP_FRAME的坐标变换（最新可用的）
        transform = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time(), rclpy.time.Duration(seconds=5))

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation
        e = quaternion_to_eular(q.x, q.y, q.z, q.w)
        theta = e[2]

        return x, y, theta

rclpy.init()
node = SendNavGoalNode()
executor = SingleThreadedExecutor()
executor.add_node(node)
executor_thread = Thread(target=executor.spin)
executor_thread.start()

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})


@app.route('/nav', methods=['POST'])
def nav():
    try:
        req : NavRequest = NavRequest(
            x = request.json.get('x'),
            y = request.json.get('y'),
            theta = request.json.get('theta'))
    except ValueError as ve:
        msg = ve.errors()[0]['msg']
        err = RuntimeError(msg)
        err.code = 400
        raise err
    x = req.x; y = req.y; theta = req.theta

    # check if the input is valid
    if x is None or y is None or theta is None:
        err = RuntimeError('invalid input: x, y, theta must be provided')
        err.code = 400
        raise err
    success, message = node.send_nav_goal(x, y, theta)
    if success:
        return jsonify({'status': 'success', 'message': message})
    else:
        err = RuntimeError(message)
        err.code = 500
        raise err
    
@app.route('/nav_sync', methods=['POST'])
def nav_sync():
    try:
        req : NavRequest = NavRequest(
            x = request.json.get('x'),
            y = request.json.get('y'),
            theta = request.json.get('theta'))
    except ValueError as ve:
        msg = ve.errors()[0]['msg']
        err = RuntimeError(msg)
        err.code = 400
        raise err
    x = req.x; y = req.y; theta = req.theta

    # check if the input is valid
    if x is None or y is None or theta is None:
        err = RuntimeError('invalid input: x, y, theta must be provided')
        err.code = 400
        raise err
    success, message = node.send_nav_goal_sync(x, y, theta)
    if success:
        return jsonify({'status': 'success', 'message': message})
    else:
        err = RuntimeError(message)
        err.code = 500
        raise err


@app.route("/nav_test", methods=['POST'])
def nav_test():
    import math
    import time
    curX, curY, curTheta = node.robot_loc()
    node.send_nav_goal_sync(curX, curY, curTheta + math.pi) # 旋转180度
    time.sleep(1)
    node.send_nav_goal_sync(curX, curY, curTheta) # 旋转回来
    return jsonify({'status': 'success', 'message': 'Nav test finished. Please check whether the robot has rotated 180 degrees and then rotated back.'})

@app.route('/initial_pose', methods=['POST'])
def initial_pose():
    try:
        req : InitialPoseRequest = InitialPoseRequest(
            x = request.json.get('x'),
            y = request.json.get('y'),
            theta = request.json.get('theta'))
    except ValueError as ve:
        msg = ve.errors()[0]['msg']
        err = RuntimeError(msg)
        err.code = 400
        raise err
    x = req.x; y = req.y; theta = req.theta
    
    if x is None or y is None or theta is None:
        err = RuntimeError('invalid input: x, y, theta must be provided')
        err.code = 400
        raise err
    
    node.send_initial_pose(x, y, theta)
    return jsonify({'status': 'success'})

@app.route('/is_moving', methods=['GET'])
def is_moving():
    # 返回机器人是否在移动
    return jsonify({'is_moving': node.is_moving()})

@app.route('/cancel_nav', methods=['POST'])
def cancel_nav():
    success, message = node.cancel_nav()
    if success:
        return jsonify({'status': 'success', 'message': message})
    else:
        err = RuntimeError(message)
        err.code = 500
        raise err

@app.route('/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok'})

@app.errorhandler(Exception)
def handle_unexpected_error(error):
    import time
    path = request.path
    method = request.method
    timestamp = int(time.time())
    code = 500 if not hasattr(error, 'code') else error.code
    message = str(error)
    return make_response(jsonify({'path': path, 'method': method, 'timestamp': timestamp, 'code': code, 'message': message}), code)

def main():
    port = 5000
    host = '0.0.0.0'
    logging.info(
        f'nav_http_server started at {host}:{port}, --map-frame: {MAP_FRAME}, --navigate-to-pose-action: {NAVIGATE_TO_POSE_ACTION}')
    serve(app, host=host, port=port)


if __name__ == '__main__':
    main()
