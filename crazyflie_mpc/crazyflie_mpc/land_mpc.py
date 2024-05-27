import rclpy
from rclpy.node import Node

from std_msgs.msg import  Empty
import time
from crazyflie_interfaces.srv import GoTo,Land,Takeoff,NotifySetpointsStop
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as Dur


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.takeoff_pub = self.create_publisher(Empty, '/all/mpc_takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/all/mpc_land', 1)
        self.takeoff = self.create_client(Takeoff, '/cf_1/takeoff')
        self.land = self.create_client(Land, '/cf_1/land')
        self.goto = self.create_client(GoTo, '/cf_1/go_to')
        self.stop = self.create_client(NotifySetpointsStop, '/cf_1/notify_setpoints_stop')

        self.takeoff_mpc()
        # t =Takeoff.Request()
        # t.height =1.0
        # duration = Dur()
        # duration.sec = 5  # 10 seconds
        # duration.nanosec = 500000000  # 0.5 seconds

        # t.duration =duration
        # self.future = self.takeoff.call_async(t)
        # rclpy.spin_until_future_complete(self, self.future)
        # time.sleep(6)
        # self.landReqest = True
        # t1 =Land.Request()
        # t1.height =1.0
        # duration = Dur()
        # duration.sec = 5  # 10 seconds
        # duration.nanosec = 500000000  # 0.5 seconds
        # t1.duration =duration
        # self.future1 = self.land.call_async(t1)
        # rclpy.spin_until_future_complete(self, self.future1)

        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
    def land_mpc(self):
        temp=Empty()
        self.land_pub.publish(temp)
    def takeoff_mpc(self):
        temp=Empty()
        self.takeoff_pub.publish(temp)

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()