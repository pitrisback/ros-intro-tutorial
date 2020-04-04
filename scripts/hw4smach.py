#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time


def timer():
    return int(time.mktime(time.gmtime()))


# state marrtino unloading to dock
class MarrUnloading2Dock(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self, outcomes=["dock1", "dock2"], output_keys=["x", "y", "yaw"]
        )

    def execute(self, ud):
        """
        """
        start_time = timer()
        end_time = timer()
        timeout = 5

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("MarrUnloading2Dock - working")
            rospy.sleep(0.5)

        if end_time / 2 == 0:
            rospy.logdebug("MarrUnloading2Dock - %s" % "dock1")
            return "dock1"
        else:
            rospy.logdebug("MarrUnloading2Dock - %s" % "dock2")
            return "dock2"


# state marrtino unloading to dock
class UR5InitialSetup(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self,
            outcomes=["ur5_init_succ", "ur5_init_fail"],
            input_keys=["ur5_task_name"],
        )

    def execute(self, ud):
        """
        """
        rospy.logdebug("Received task to do %s" % ud.ur5_task_name)

        start_time = timer()
        end_time = timer()
        timeout = 2

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("UR5InitialSetup - working")
            rospy.sleep(0.5)

        if end_time / 10 != 0:
            return "ur5_init_succ"
        else:
            return "ur5_init_fail"


def main():
    rospy.init_node("smach_hw4", log_level=rospy.DEBUG)

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=["caricadock1", "caricadock2", "con1_failed"])
    sm_top.userdata.top_task_name = "task1"

    # Open the container
    with sm_top:

        # Create the sub concurrent SMACH state machine to prepare for loading
        sm_con_pre_loading = smach.Concurrence(
            outcomes=["con_dock1", "con_dock2", "con1_fail"],
            default_outcome="con1_fail",
            outcome_map={
                "con_dock1": {
                    "UR5INITIALSETUP": "ur5_init_succ",
                    "MARRUNLOADING2DOCK": "dock1",
                },
                "con_dock2": {
                    "UR5INITIALSETUP": "ur5_init_succ",
                    "MARRUNLOADING2DOCK": "dock2",
                },
            },
            input_keys=["con_task_name"],
        )

        # Open the container
        with sm_con_pre_loading:
            # Add STATES to the container
            smach.Concurrence.add(
                "UR5INITIALSETUP",
                UR5InitialSetup(),
                remapping={"ur5_task_name": "con_task_name"},
            )
            smach.Concurrence.add("MARRUNLOADING2DOCK", MarrUnloading2Dock())

        # add the concurret state machine as state in the top machine
        smach.StateMachine.add(
            "PRE_LOADING",
            sm_con_pre_loading,
            transitions={
                "con_dock1": "caricadock1",
                "con_dock2": "caricadock2",
                "con1_fail": "con1_failed",
            },
            remapping={"con_task_name": "top_task_name"},
        )

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == "__main__":
    main()
