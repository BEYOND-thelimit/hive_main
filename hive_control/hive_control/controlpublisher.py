import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math

def quaternion_to_euler(quaternion):
    """
    쿼터니언에서 오일러 각(yaw)으로 변환하는 함수입니다.
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion(q)
    return yaw

def euler_from_quaternion(q):
    """
    주어진 쿼터니언으로부터 오일러 각(롤, 피치, 요)을 계산합니다.
    """
    (x, y, z, w) = q
    # 오일러 각을 계산합니다.
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

def checkLinearLimitVelocity(velocity, max_velocity=0.7):
    """
    속도의 최대 값을 제한하는 함수입니다.
    """
    return max(min(velocity, max_velocity), -max_velocity)

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(String, 'target_position', self.cmd_callback, 10)
        self.publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        
        # 로봇의 초기 현재 위치 및 방향을 설정합니다. 이는 나중에 업데이트 됩니다.
        self.curr_pose = np.array([0.0, 0.0])
        self.curr_ori = Quaternion()
        self.initialized = False  # 초기 명령 수신 여부를 확인하는 플래그

    def cmd_callback(self, msg):
        """
        'target_position' 토픽의 콜백 함수입니다. 목표 위치와 방향을 업데이트합니다.
        """
        data = msg.data.split(',')
        if len(data) == 3:
            self.target_pose = np.array([float(data[0]), float(data[1])])
            self.target_yaw = float(data[2])
            self.initialized = True  # 초기 명령 수신
            self.get_logger().info(f'Updated target pose: {self.target_pose}, yaw: {self.target_yaw}')

    def odom_callback(self, msg):
        """
        '/robot1/odom' 토픽의 콜백 함수입니다. 현재 위치와 방향을 업데이트하고 제어합니다.
        """
        self.curr_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.curr_ori = msg.pose.pose.orientation
        if not self.initialized:
            # 초기 명령이 수신되기 전까지 현재 위치와 방향을 목표로 설정합니다.
            self.target_pose = self.curr_pose
            self.target_yaw = quaternion_to_euler(self.curr_ori)
        self.control()

    def control(self):
        """
        현재 위치에서 목표 위치까지의 제어 명령을 계산하고 발행합니다. 
        목표 위치에 도달한 후에는 목표 yaw를 맞추기 위해 회전합니다.
        """
        yaw_curr = quaternion_to_euler(self.curr_ori)  # 현재 로봇의 방향(yaw)을 계산
        x_error = self.target_pose[0] - self.curr_pose[0]
        y_error = self.target_pose[1] - self.curr_pose[1]
        
        distance_error = math.sqrt(x_error**2 + y_error**2)  # 목표까지의 거리를 계산
        target_angle = math.atan2(y_error, x_error)  # 목표 방향을 계산
        angle_to_target = math.atan2(math.sin(target_angle - yaw_curr), math.cos(target_angle - yaw_curr))  # 현재 방향에서 목표 방향까지의 최소 회전 각도를 계산

        if distance_error > 0.05:  # 아직 목표 위치에 도달하지 않았다면 이동합니다.
            v = checkLinearLimitVelocity(0.5 * distance_error)  # 속도를 거리 오차에 비례하게 조절
            v_yaw = checkLinearLimitVelocity(2.0 * angle_to_target)  # 목표 방향으로의 회전을 조금 적용
        else:  # 목표 위치에 도달했다면 목표 yaw에 맞춥니다.
            v = 0.0  # 위치에 도달했으므로 전진 속도를 0으로 설정
            yaw_error = (self.target_yaw - yaw_curr + math.pi) % (2 * math.pi) - math.pi  # 목표 yaw와 현재 yaw 사이의 최소 오차를 계산
            v_yaw = checkLinearLimitVelocity(1.0 * yaw_error)  # 목표 yaw를 정확히 맞추기 위해 회전 속도를 조절
            if abs(yaw_error) < 0.05:  # yaw 오차가 충분히 작다면 멈춤
                v_yaw = 0.0
                self.get_logger().info('Yaw aligned. Arrived at the target position and orientation.')

        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = v_yaw

        self.publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
