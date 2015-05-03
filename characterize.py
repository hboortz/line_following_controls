#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import cv2

from geometry_msgs.msg import Twist

class Follower(object):
    def __init__(self):
        self.bathtub = 0
        self.previous_error = 0
        self.first_term = True

    def mainloop(self, cap):
        for i in range(10):
            _, frame = cap.read()
            cv2.imshow('frame',frame)
            _, frame = cap.read()

            # Threshold to get black line
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_bound = np.array([0,0,0])
            upper_bound = np.array([100,100,100])
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            middle = mask.shape[0]/2
            res = cv2.bitwise_and(frame,frame, mask=mask)

            # Get contours and centroid
            contours, heirarchy = cv2.findContours(mask, 1, 2)
            cnt = contours[0]
            momt = cv2.moments(cnt)
            # print momt
            try:
                cx = int(momt['m10']/momt['m00'])
                cy = int(momt['m01']/momt['m00'])
                diff = middle - cx
                print "Current delta x is", diff
                ang_vel = 0.17
                print "Angular vel command was", ang_vel
                self.control_motors(ang_vel)
                print diff
            except ZeroDivisionError:
                pass

            # display contours
            # raw_input()
            cv2.imshow('mask',mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    def control_motors(self, turn_speed):
        # first thing, init a node!
        rospy.init_node('move')

        # publish to cmd_vel
        p = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        # create a twist message, fill in the details
        twist = Twist()
        x_speed = 0.1  # 0.1 m/s
        twist.linear.x = x_speed;                   # our forward speed
        twist.linear.y = 0;
        twist.angular.z = turn_speed # positive is left

        # announce move, and publish the message
        rospy.loginfo("Moving at speed %d" % (turn_speed))
    #    rospy.loginfo("About to be moving forward!")
        for i in range(10):
            p.publish(twist)
            rospy.sleep(0.1) # 30*0.1 = 3.0
        # create a new message
    #    twist = Twist()

        # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    #    rospy.loginfo("Stopping!")
    #    p.publish(twist)

    def calc_turn_speed(self, diff):
        max_diff = 240.0
        Kp = 0.2
        Ki = 0.2        
        Kd = 0.1
        if self.first_term:
            Kd = 0  # don't know derivative term for the very first one
        error = diff/max_diff
        delta = self.previous_error - error
        self.bathtub += error
        act = Kp*error + Ki*self.bathtub + Kd*delta
        self.previous_error = error
        return act

    def cleanup(self, cap):
        cap.release()
        cv2.destroyAllWindows()


# def calc_turn_speed(diff):
#     max_diff = 240.0
#     scale = 0.4
#     return diff/max_diff * scale


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    pinky = Follower()
    pinky.mainloop(cap)
    pinky.cleanup(cap)
