#!/usr/bin/env python
# license removed for brevity
import sys,tty,termios
import curses

import rospy
from mj_models.msg import MushrControl
from mj_models.msg import MushrObservation
from mj_models.srv import MushrFeedback,MushrFeedbackResponse

screen = None
control = MushrControl()
observation = MushrObservation()
robot_name = "mushr"
velocity_increase = 0.1
steering_increase = 0.1

def handle_service(req):
    global control
    global observation
    observation = req.observation
    return MushrFeedbackResponse(control)

def reset():
    global control
    control.steering_angle.data = 0
    control.velocity.data = 0

def keyboard_terminal():
    global control
    global observation

    feedback_service = rospy.Service('/' + robot_name + '/feedback_service', MushrFeedback, handle_service)
    rospy.init_node('keyboard_terminal', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    screen = curses.initscr()

    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    screen.addstr("Usage: \n")
    screen.addstr("\tUP: w or up arrow \n")
    screen.addstr("\tDOWN: s or down arrow \n")
    screen.addstr("\tRIGHT: d or right arrow \n")
    screen.addstr("\tLEFT: a or left arrow \n")
    screen.addstr("\tSTART/STOP: spacebar \n")
    # screen.addstr("")
    reset()
    while not rospy.is_shutdown():
        c = screen.getch()
        if c == curses.KEY_UP or c == ord("w"):
            # pub.publish()
            control.velocity.data += velocity_increase
        elif c == curses.KEY_DOWN or c == ord("s"):
            control.velocity.data -= velocity_increase
        elif c == curses.KEY_LEFT or c == ord("a"):
            control.steering_angle.data += steering_increase
        elif c == curses.KEY_RIGHT or c == ord("d"):
            control.steering_angle.data -= steering_increase
        elif c == ord("r"):
            reset()
            pub_start.publish(start)
        elif c == 27:
            reset()
        screen.addstr(7, 5, "\tSPEED: " + str(control.velocity.data) + " \n")
        screen.addstr(8, 5, "\tSTEER: " + str(control.steering_angle.data) + " \n")
        screen.addstr(8, 5, "\tObservation: " + str(observation) + " \n")

        screen.refresh()
    curses.nocbreak()
    screen.keypad(False)
    curses.echo()
    curses.endwin()

if __name__ == '__main__':
    try:
        keyboard_terminal()
    except rospy.ROSInterruptException:
        # 
        pass