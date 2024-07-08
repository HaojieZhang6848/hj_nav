from rclpy.node import Node
import rclpy
import tf2_ros
from waitress import serve
try:
    from .utils import quaternion_to_eular  # for ros2 run
except:
    from utils import quaternion_to_eular  # for direct run
from flask import Flask, jsonify, make_response, request
from multiprocessing.shared_memory import SharedMemory
import time
import atexit
from flask_cors import CORS

# 设置日志级别和格式
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def wait_for_shm(name):
    def close_shm(shmm):
        if shmm and shmm._name:
            shmm.close()
            shmm.unlink()
    while True:
        try:
            shm = SharedMemory(name=name)
            break
        except:
            logging.info('Waiting for shared memory red_detected......')
            time.sleep(1)
    atexit.register(close_shm, shm)
    return shm

shm = wait_for_shm('red_detected')

def get_red_detected():
    # 如果最近的三次检测中有两次检测到红色物体，则认为检测到了红色物体
    return sum(shm.buf[:3]) >= 2

class RedObjServer(Node):
    def __init__(self):
        super().__init__('red_obj_server')
        self.get_logger().info('Red Obj Server started')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
    
    
    def get_loc(self, frame: str):
        try:
            trans = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time(), rclpy.time.Duration(seconds=5))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            e = quaternion_to_eular(q.x, q.y, q.z, q.w)
            theta = e[2]
        except:
            x = 0
            y = 0
            theta = 0
        
        return x, y, theta
        
    
    def red_loc(self):
        # 检查red_detector是否检测到红色物体
        if not get_red_detected():
            raise Exception('Red object not detected')
        red_x, red_y, red_theta = self.get_loc('red_object')
        car_1_x, car_1_y, car_1_theta = 0.0, 0.0, 0.0
        car_2_x, car_2_y, car_2_theta = 0.0, 0.0, 0.0
        
        return red_x, red_y, red_theta, car_1_x, car_1_y, car_1_theta, car_2_x, car_2_y, car_2_theta           
   
rclpy.init()
node = RedObjServer()     
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

@app.route('/red_loc', methods=['GET'])
def red_loc():
    try:
        red_x, red_y, red_theta, car_1_x, car_1_y, car_1_theta, car_2_x, car_2_y, car_2_theta = node.red_loc()
        dat = {
            "red_object": {
                "x": red_x,
                "y": red_y,
                "theta": red_theta
            },
            "another_car_1": {
                "x": car_1_x,
                "y": car_1_y,
                "theta": car_1_theta
            },
            "another_car_2": {
                "x": car_2_x,
                "y": car_2_y,
                "theta": car_2_theta
            }
        }
        return jsonify(dat)
    except Exception as e:
        e.code = 404 if 'not detected' in str(e) else 500
        raise e

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
    host = '0.0.0.0'
    port = 5002
    logging.info(f'Red Obj Server started at {host}:{port}')
    serve(app, host=host, port=port)


if __name__ == '__main__':
    main()
