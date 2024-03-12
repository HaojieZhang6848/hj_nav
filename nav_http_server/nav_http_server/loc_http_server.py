from flask import Flask, jsonify, request, make_response
from waitress import serve
from argparse import ArgumentParser
import rclpy
import tf2_ros
try:
    from .utils import quaternion_to_eular  # for ros2 run
except:
    from utils import quaternion_to_eular  # for python3 nav_http_server.py
import logging
from datetime import datetime
from flask_cors import CORS

# 设置日志级别和格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 解析--map-frame,--robot-frame和--navigate-to-pose-action三个参数
parser = ArgumentParser()
parser.add_argument('--map-frame', default='map')
parser.add_argument('--robot-frame', default='base_link')
args, unknown = parser.parse_known_args()

MAP_FRAME = args.map_frame
ROBOT_FRAME = args.robot_frame


class RobotLocNode(rclpy.node.Node):
    def __init__(self):
        ts = datetime.now().timestamp()
        super().__init__('loc_http_server' + str(ts).replace('.', ''))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

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
node = RobotLocNode()

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})


@app.route('/loc', methods=['GET'])
def loc():
    # 获取机器人当前位置
    x, y, theta = node.robot_loc()
    return jsonify({'x': x, 'y': y, 'theta': theta})

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
    host = '0.0.0.0'
    port = 5001
    logging.info(f'loc_http_server started at {host}:{port}, --map-frame={MAP_FRAME}, --robot-frame={ROBOT_FRAME}')
    serve(app, host='0.0.0.0', port=5001)


if __name__ == '__main__':
    main()
