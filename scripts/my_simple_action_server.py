#! /usr/bin/env python2

import rospy
import actionlib

from intro_tutorial.msg import FibonacciAction
from intro_tutorial.msg import FibonacciFeedback, FibonacciResult


class FibonacciServer(object):
    # create messages that are used to publish feedback/result
    _feedback = FibonacciFeedback()
    _result = FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            FibonacciAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

    def execute_cb(self, goal):
        """Function that runs everytime a new goal is received
        """
        # helper variables
        rosrate = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        # publish info to the console for the user
        rospy.loginfo(
            "%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i"
            % (
                self._action_name,
                goal.order,
                self._feedback.sequence[0],
                self._feedback.sequence[1],
            )
        )

        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # update the sequence
            self._feedback.sequence.append(
                self._feedback.sequence[i] + self._feedback.sequence[i - 1]
            )

            # publish the feedback on the feedback channel provided by the action server
            self._as.publish_feedback(self._feedback)

            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            rosrate.sleep()

        if success:
            # set the result
            self._result.sequence = self._feedback.sequence
            rospy.loginfo("%s: Succeeded" % self._action_name)
            # notifies the action client that the goal is complete by calling set_succeeded
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("fibonacci")
    server = FibonacciServer(rospy.get_name())
    rospy.spin()
