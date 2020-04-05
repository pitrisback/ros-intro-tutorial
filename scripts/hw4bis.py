#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time


def timer():
    return int(time.mktime(time.gmtime()))


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["outcome_foo_1"],
            input_keys=["foo_counter_in"],
            output_keys=["foo_counter_out"],
        )

    def execute(self, ud):
        rospy.loginfo("Executing state FOO")
        rospy.loginfo("FOO Counter in = %f" % ud.foo_counter_in)
        ud.foo_counter_out = ud.foo_counter_in + 1
        #  rospy.loginfo("FOO Counter out = %f" % ud.foo_counter_out)
        return "outcome_foo_1"


class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["outcome_bar_1", "outcome_bar_2"],
            input_keys=["bar_counter_in"],
            output_keys=["bar_counter_out"],
        )

    def execute(self, ud):
        rospy.loginfo("Executing state BAR")
        rospy.loginfo("BAR Counter = %f" % ud.bar_counter_in)
        if ud.bar_counter_in < 3:
            ud.bar_counter_out = ud.bar_counter_in + 1
            return "outcome_bar_1"
        else:
            return "outcome_bar_2"


def main():
    rospy.init_node("smach_example_state_machine")

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=["outcome_bar_2"]
        #  , input_keys=["sm_input"]
        #  , output_keys=["sm_output"]
    )
    sm.userdata.sm_input = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "FOO",
            Foo(),
            transitions={"outcome_foo_1": "BAR"},
            remapping={"foo_counter_in": "sm_input", "foo_counter_out": "sm_data"},
        )
        smach.StateMachine.add(
            "BAR",
            Bar(),
            transitions={"outcome_bar_1": "BAR"},
            remapping={"bar_counter_in": "sm_data", "bar_counter_out": "sm_data"},
        )

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == "__main__":
    main()
