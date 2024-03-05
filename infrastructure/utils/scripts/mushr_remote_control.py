#!/usr/bin/env python
# license removed for brevity
import curses
import rospy
# from prx_models.msg import MushrControl
from ackermann_msgs.msg import AckermannDriveStamped
from prx_models.msg import MushrObservation
from prx_models.srv import MushrFeedback,MushrFeedbackResponse
from std_msgs.msg import Empty

class KeyboardControlService:
    def __init__(self):
        self.control = AckermannDriveStamped()
        self.observation = MushrObservation()

        self.velocity_increase = 0.1
        self.steering_increase = 0.25

        # self.feedback_service = rospy.Service(rospy.get_namespace() + 'feedback_service', MushrFeedback, self.handle_service)
        rospy.init_node('keyboard_terminal', anonymous=True)

        self.ctrl_pub = rospy.Publisher('/mushr/mux/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
        self.reset_pub = rospy.Publisher(rospy.get_namespace() + 'reset', Empty, queue_size=10)

        self.rate = rospy.Rate(10)

    def reset_control(self):
        self.control.drive.steering_angle = 0
        self.control.drive.speed = 0
    
    def keyboard_terminal(self):
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.keypad(True)
        self.reset_control()

        while not rospy.is_shutdown():
            key = screen.getch()
            if key == curses.KEY_UP or key == ord('w'):
                self.control.drive.speed += self.velocity_increase
            elif key == curses.KEY_DOWN or key == ord('s'):
                self.control.drive.speed -= self.velocity_increase
            elif key == curses.KEY_LEFT or key == ord('a'):
                self.control.drive.steering_angle += self.steering_increase
            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.control.drive.steering_angle -= self.steering_increase
            elif key == ord('r'):
                self.reset_pub.publish(Empty())
                self.reset_control()
            elif key == ord('q'):
                break
            self.ctrl_pub.publish(self.control);
            screen.addstr(10, 0, 'Velocity: ' + str(self.control.drive.speed) + ' Steering: ' + str(self.control.drive.steering_angle) + '\n')
            screen.addstr(11, 0, "Position: " + str(self.observation.pose.position.x) + " " + str(self.observation.pose.position.y) + " " + str(self.observation.pose.position.z) + '\n')
            screen.refresh()


        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()

    # def handle_service(self,req):
    #     self.observation = req.observation
    #     return MushrFeedbackResponse(self.control)
    
if __name__ == '__main__':
    try:
        keyboard_control_service = KeyboardControlService()
        keyboard_control_service.keyboard_terminal()
    except rospy.ROSInterruptException:
        pass
