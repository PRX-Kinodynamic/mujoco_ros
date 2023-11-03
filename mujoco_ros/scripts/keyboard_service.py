#!/usr/bin/env python
# license removed for brevity
import curses
import rospy
from mj_models.msg import MushrControl
from mj_models.msg import MushrObservation
from mj_models.srv import MushrFeedback,MushrFeedbackResponse
from std_msgs.msg import Empty

class KeyboardControlService:
    def __init__(self):
        self.control = MushrControl()
        self.observation = MushrObservation()

        self.velocity_increase = 0.1
        self.steering_increase = 0.1

        self.feedback_service = rospy.Service(rospy.get_namespace() + 'feedback_service', MushrFeedback, self.handle_service)
        rospy.init_node('keyboard_terminal', anonymous=True)

        self.reset_pub = rospy.Publisher(rospy.get_namespace() + 'reset', Empty, queue_size=10)

        self.rate = rospy.Rate(10)

    def reset_control(self):
        self.control.steering_angle.data = 0
        self.control.velocity.data = 0
    
    def keyboard_terminal(self):
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.keypad(True)
        self.reset_control()

        while not rospy.is_shutdown():
            key = screen.getch()
            if key == curses.KEY_UP or key == ord('w'):
                self.control.velocity.data += self.velocity_increase
            elif key == curses.KEY_DOWN or key == ord('s'):
                self.control.velocity.data -= self.velocity_increase
            elif key == curses.KEY_LEFT or key == ord('a'):
                self.control.steering_angle.data += self.steering_increase
            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.control.steering_angle.data -= self.steering_increase
            elif key == ord('r'):
                self.reset_pub.publish(Empty())
                self.reset_control()
            elif key == ord('q'):
                break
            screen.addstr(10, 0, 'Velocity: ' + str(self.control.velocity.data) + ' Steering: ' + str(self.control.steering_angle.data) + '\n')
            screen.addstr(11, 0, "Position: " + str(self.observation.pose.position.x) + " " + str(self.observation.pose.position.y) + " " + str(self.observation.pose.position.z) + '\n')
            screen.refresh()


        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()

    def handle_service(self,req):
        self.observation = req.observation
        return MushrFeedbackResponse(self.control)
    
if __name__ == '__main__':
    try:
        keyboard_control_service = KeyboardControlService()
        keyboard_control_service.keyboard_terminal()
    except rospy.ROSInterruptException:
        pass