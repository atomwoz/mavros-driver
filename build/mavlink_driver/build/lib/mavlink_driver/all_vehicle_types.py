import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float64
from rclpy.qos import ReliabilityPolicy, QoSProfile

minimal_client = None

SPEED = 1.5


class MavrosUniversalVehicleDriver(Node):

    def __init__(self):
        super().__init__('mavros_universal_driver')
        
        self.req_speed = Vector3()
        self.rel_alt = -1
        self.point_flag = False

        self.arm_client = self.create_client(CommandBool,'/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Arming service not available, waiting again...')
        
        self.takeoff_client = self.create_client(CommandTOL,'/mavros/cmd/takeoff')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Takeoff service not available, waiting again...')
        
        self.command_client = self.create_client(CommandLong,'/mavros/cmd/command')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Other service not available, waiting again...')
        
        self.setmode_client = self.create_client(SetMode,'/mavros/set_mode')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Other service not available, waiting again...')
        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel',16)
        self.alt_subscribtion = self.create_subscription(Float64,'/mavros/global_position/rel_alt',self.alt_callback,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.point_subscribtion = self.create_subscription(Vector3,'/mavlink_driver/point',self.point_callback,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))

        self.arm_message = CommandBool.Request()
        self.takeoff_message = CommandTOL.Request()
        self.command_message = CommandLong.Request()
        self.geo_message = TwistStamped()
        self.setmode_message = SetMode.Request()

    def point_callback(self,data):
        self.point_flag = True
        self.req_speed = data

    def alt_callback(self,data):
        self.rel_alt = float(data.data)

    def info(self,msg):
        self.get_logger().info(str(msg))

    def arm_vehicle(self):
        self.arm_message.value = True
        self.info("OD ŚMIGŁA!!! Arming vehicle")
        self.future_arm = self.arm_client.call_async(self.arm_message)

    def takeoff(self,min_pitch=0.0,yaw=0.0,altitude=2.0):
        self.takeoff_message.min_pitch = min_pitch
        self.takeoff_message.yaw = yaw
        self.takeoff_message.altitude = altitude
        self.info("Request takeoff")
        self.future_takeoff = self.takeoff_client.call_async(self.takeoff_message)

    def guided(self):
        self.command_message.command = 176
        #self.command_message.param1 = 216.0
        self.command_message.param1 = 88.0

        self.setmode_message.base_mode = 0
        self.setmode_message.custom_mode = "GUIDED"
        self.info("Request guided mode")
        self.future_setmode = self.setmode_client.call_async(self.setmode_message)

    def subscribe_altitude(self):
        self.command_message.command = 511
        self.command_message.param1 = 33.0
        self.info(self.command_client.call_async(self.command_message))

    def error(self,msg):
        self.get_logger().error(str(msg))
    
    def warn(self,msg):
        self.get_logger().warn(str(msg))
    
    def set_velocity(self,x=0,y=0,z=0,ax=0,ay=0,az=0,speed=1.5):
        self.twist = self.geo_message.twist
        self.twist.linear.x = float(x) * speed
        self.twist.linear.y = float(y) * speed
        self.twist.linear.z = float(z) * speed
        self.twist.angular.x = float(ax) * speed
        self.twist.angular.y = float(ay) * speed
        self.twist.angular.z = float(az) * speed
        self.velocity_publisher.publish(self.geo_message)

def wait_for_result(eq,msg_ok,msg_error="",error_fun=None):
    global minimal_client
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if eq(minimal_client)._done:
            if eq(minimal_client)._result.success:
                minimal_client.info(msg_ok)
            else:
                if msg_error != "":
                    minimal_client.error(msg_error)
                if error_fun != None:
                    error_fun()
            break

def wait_for_point():
    global minimal_client
    while rclpy.ok() and not minimal_client.point_flag:
        rclpy.spin_once(minimal_client)
    minimal_client.point_flag = False

def ask_velocity():
    global minimal_client
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        vel = minimal_client.req_speed
        minimal_client.set_velocity(x=vel.x,y=vel.y,z=vel.z,speed=SPEED)

def wait_for_takeoff(alt=2):
    global minimal_client
    while minimal_client.rel_alt <= alt-(0.25*alt) and rclpy.ok():
        rclpy.spin_once(minimal_client)
        #minimal_client.info(minimal_client.rel_alt)
    minimal_client.info("Vehicle takeoffed to " + str(minimal_client.rel_alt))

def wait_for_mode(eq,msg_ok,msg_error="",error_fun=None):
    global minimal_client
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if eq(minimal_client)._done:
            if eq(minimal_client)._result.mode_sent:
                minimal_client.info(msg_ok)
            else:
                if msg_error != "":
                    minimal_client.error(msg_error)
                if error_fun != None:
                    error_fun()
            break

def main(args=None):
    global minimal_client
    rclpy.init(args=args)
    minimal_client = MavrosUniversalVehicleDriver()
    minimal_client.info("Waiting for first message at /mavlink_driver/point Vector3")
    wait_for_point()
    
    minimal_client.subscribe_altitude()
    
    minimal_client.guided()
    wait_for_mode(lambda a : a.future_setmode,"Vehicle in GUIDED mode",error_fun = lambda : minimal_client.warn("Vehicle's mode wasn't changed but we can go ahead"))

    minimal_client.arm_vehicle()
    wait_for_result(lambda a : a.future_arm,"Vehicle ARMED","Vehicle wasn't armed !!!!")

    minimal_client.takeoff()
    wait_for_result(lambda a : a.future_takeoff,"Vehicle was asked to takeoff, will fly :)","FC Rejected takeoff !!!!")
    wait_for_takeoff() 
    
    ask_velocity()
    
    
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
