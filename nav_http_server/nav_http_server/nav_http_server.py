from flask import Flask, request, jsonify, make_response
from waitress import serve
from argparse import ArgumentParser
from rclpy.action import ActionClient
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime
try:
    from .utils import eular_to_quaternion  # for ros2 run
    from .models import NavRequest, InitialPoseRequest
except:
    from utils import eular_to_quaternion  # for python3 nav_http_server.py
    from models import NavRequest, InitialPoseRequest
import logging
from threading import Thread, Semaphore
from flask_cors import CORS
from geometry_msgs.msg import PoseWithCovarianceStamped

# 设置日志级别和格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 解析--map-frame和--navigate-to-pose-action两个参数
parser = ArgumentParser()
parser.add_argument('--map-frame', default='map')
parser.add_argument('--navigate-to-pose-action', default='/navigate_to_pose')
args, unknown = parser.parse_known_args()

MAP_FRAME = args.map_frame
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
        self.last_goal = None
        self.is_moving = False
        self.lock = Semaphore(1)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
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
        
    def send_nav_goal(self, x, y, theta):
        # 如果多次以相同的参数调用send_nav_goal，则不会发送新的目标
        if self.last_goal is not None:
            x_last, y_last, theta_last = self.last_goal
            dist = (x-x_last)**2 + (y-y_last)**2
            rot = (theta-theta_last)**2
            if dist < DIST_THRESHOLD and rot < ROT_THRESHOLD:
                return True, 'goal not changed'

        self.last_goal = (x, y, theta)
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
        
        # 这里的lock的目的是：当导航被抢占(前一个导航还没结束，又有新的导航请求)时
        # 确保旧的导航将is_moving设置为False后，新的导航才将is_moving设置为True
        # 如果上述两个过程的顺序颠倒，那么is_moving的值会错误
        self.lock.acquire()
        
        rclpy.spin_until_future_complete(self, goal_handle_future, executor=self.executor)
        goal_handle = goal_handle_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False, 'goal rejected'
        
        self.get_logger().info('Move started')
        self.is_moving = True
        
        def wait_move_stop():
            rsp_f = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, rsp_f, executor=self.executor)
            self.get_logger().info('Move stopped')
            self.is_moving = False
            self.lock.release()
            
        t = Thread(target=wait_move_stop)
        t.start()
        
        return True, 'goal accepted'


rclpy.init()
node = SendNavGoalNode()
# t = Thread(target=node.executor.spin)
# t.start()

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
    return jsonify({'is_moving': node.is_moving})

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
