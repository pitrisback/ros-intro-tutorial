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
            self,
            outcomes=["marr_succ", "marr_fail"],
            output_keys=["marr_dock", "marr_x", "marr_y", "marr_yaw"],
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
            ud.marr_dock = 1
            return "marr_succ"
        else:
            rospy.logdebug("MarrUnloading2Dock - %s" % "dock2")
            ud.marr_dock = 2
            return "marr_succ"


# state marrtino unloading to dock
class UR5InitialSetup(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self, outcomes=["ur5_init_succ", "ur5_init_fail"],
        )

    def execute(self, ud):
        """
        """
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


class UR5LoadObjects(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self,
            outcomes=["ur5_load_succ", "ur5_load_fail"],
            input_keys=[
                "ur5_task_name",
                "ur5_dock",
                "ur5_load_x",
                "ur5_load_y",
                "ur5_load_yaw",
            ],
        )

    def execute(self, ud):
        """
        """
        rospy.logdebug("Received task to do %s" % ud.ur5_task_name)

        start_time = timer()
        end_time = timer()
        timeout = 5

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("UR5LoadObjects - working")
            rospy.sleep(0.5)

        if end_time / 10 != 0:
            return "ur5_load_succ"
        else:
            return "ur5_load_fail"


def main():
    rospy.init_node("smach_hw4", log_level=rospy.DEBUG)

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=["ok", "fail", 'pre_load_failed'])
    sm_top.userdata.top_task_name = "task2"

    # Open the container
    with sm_top:

        # Create the sub concurrent SMACH state machine to prepare for loading
        sm_con_pre_loading = smach.Concurrence(
            outcomes=["con1_succ", "con1_fail"],
            default_outcome="con1_fail",
            outcome_map={
                "con1_succ": {
                    "UR5INITIALSETUP": "ur5_init_succ",
                    "MARRUNLOADING2DOCK": "marr_succ",
                }
            },
        )

        # Open the container
        with sm_con_pre_loading:
            # Add STATES to the container
            smach.Concurrence.add(
                "UR5INITIALSETUP", UR5InitialSetup(),
            )
            smach.Concurrence.add("MARRUNLOADING2DOCK", MarrUnloading2Dock())

        # add the concurrent state machine as state in the top machine
        smach.StateMachine.add(
            "PRE_LOADING",
            sm_con_pre_loading,
            transitions={
                "con1_succ": "UR5LOADOBJECTS",
                "con1_fail": "pre_load_failed",
            },
            remapping={"marr_dock": "top_dock",},
        )

        #  add the state to load the marrtino
        smach.StateMachine.add(
            "UR5LOADOBJECTS",
            UR5LoadObjects(),
            transitions={"ur5_load_succ": "ok", "ur5_load_fail": "fail",},
            remapping={"ur5_task_name": "top_task_name", "ur5_dock": "top_dock",},
        )

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == "__main__":
    main()
