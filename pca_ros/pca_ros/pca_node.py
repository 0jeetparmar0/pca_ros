import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import adafruit_pca9685
from adafruit_servokit import ServoKit
import serial
import board
import busio

global throttle_input, steering_input, right_steer_angle, left_steer_angle, max_throttle, min_throttle

kit = ServoKit(channels=16, address=0x40)

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

#kit.set_pwm_freq(50)
#pca.setPWMFreq(50)
pca.frequency = 100


right_steer_angle=135
left_steer_angle=30
max_throttle=125
min_throttle= 65
throttle_input = 90
steering_input = 85

kit.servo[0].angle = throttle_input
time.sleep(1)

kit.servo[1].angle = steering_input

bs1=180-throttle_input
kit.servo[2].angle = bs1



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        throttle=msg.linear.x
        steering=msg.angular.z
  #      self.get_logger().info('Y AXIS: "%s"' % yaxis)
 #       self.get_logger().info('X AXIS: "%s"' % xaxis)
        self.get_logger().info('Throttle: "%s"' % throttle)
        self.get_logger().info('Steering: "%s"' % steering)
        
        oldstrvalue = float(steering)
        
        
        oldthrvalue = float(throttle)

        if oldstrvalue <0:
            oldstrvalue = -oldstrvalue
            
            newstrvalue=int(steering_input+(oldstrvalue*((right_steer_angle-steering_input)/3)))
        else:
            newstrvalue=int(steering_input-(oldstrvalue*((steering_input-left_steer_angle)/3)))
            
 #       if oldthrvalue <0:
  #          oldthrvalue = oldthrvalue + 1

        if newstrvalue >right_steer_angle:
            newstrvalue = right_steer_angle
        if newstrvalue <left_steer_angle:
            newstrvalue = left_steer_angle           
       
       
        oldthrrange = 2
        newthrrange = max_throttle
    -min_throttle
        newthrvalue = int(((oldthrvalue) * (newthrrange)+90))

        if newthrvalue >max_throttle
    :
            newthrvalue = max_throttle
         

        if newthrvalue <min_throttle:
            newthrvalue = min_throttle 

        move_robot(newthrvalue, newstrvalue)
        
    def convertscales(oldvalue, oldmax, oldmin, newmax, newmin):
        oldrange = (oldmax - oldmin)  
        newrange = (newmax - newmin)  
        newvalue = (((oldvalue - oldmin) * newrange) / oldrange) + newmin
        return(newvalue)

        
def move_robot(thrnum,strnum):
    print("Moving Robot:  Throttle="+str(thrnum)+" ,  Steering="+str(strnum))
    kit.servo[0].angle = thrnum

    kit.servo[1].angle = strnum
    if strnum>steering_input:
        bs1=steering_input-(strnum-steering_input)
    else:
        bs1= steering_input+(steering_input-strnum)

    print(bs1)
    kit.servo[2].angle = bs1        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()