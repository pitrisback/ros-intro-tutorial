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
            outcomes=["marr_unloading2dock_succ", "marr_unloading2dock_fail"],
            output_keys=["marr_unloading2dock_dock", "marr_unloading2dock_xy_yaw"],
        )

    def execute(self, ud):
        """
        """
        start_time = timer()
        end_time = timer()
        timeout = 2

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("MarrUnloading2Dock - working")
            rospy.sleep(0.28754)

        ud.marr_unloading2dock_xy_yaw = (1, 1, 3)

        if end_time % 100 == 0:
            ud.marr_unloading2dock_dock = -1
            return "marr_unloading2dock_fail"

        if end_time % 2 == 0:
            rospy.logdebug("MarrUnloading2Dock - %s" % "dock1")
            ud.marr_unloading2dock_dock = 1
            return "marr_unloading2dock_succ"
        else:
            rospy.logdebug("MarrUnloading2Dock - %s" % "dock2")
            ud.marr_unloading2dock_dock = 2
            return "marr_unloading2dock_succ"


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
        timeout = 1

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("UR5InitialSetup - working")
            rospy.sleep(0.26532)

        if end_time % 100 != 0:
            return "ur5_init_succ"
        else:
            return "ur5_init_fail"


# state marrtino loading by the UR5
class UR5LoadObjects(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self,
            outcomes=["ur5_load_succ", "ur5_load_fail"],
            input_keys=["ur5_load_task_name", "ur5_load_dock", "ur5_load_xy_yaw"],
            output_keys=["ur5_load_task_done"],
        )

    def execute(self, ud):
        """
        """
        rospy.logdebug("Received task to do %s" % ud.ur5_load_task_name)
        rospy.logdebug("Marrtino is in %s" % ud.ur5_load_dock)
        xy_yaw = ud.ur5_load_xy_yaw
        rospy.logdebug("Received marrtino pose x %f y %f yaw %f" % xy_yaw)

        start_time = timer()
        end_time = timer()
        timeout = 2

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("UR5LoadObjects - working")
            rospy.sleep(0.254321)

        if end_time % 3 == 0:
            rospy.logdebug("UR5LoadObjects - DONE!!!")
            ud.ur5_load_task_done = True
        else:
            rospy.logdebug("UR5LoadObjects - Still some to load, come back later")
            ud.ur5_load_task_done = False

        if end_time % 100 != 0:
            return "ur5_load_succ"
        else:
            return "ur5_load_fail"


# class for martino that goes from the docks to unloading station
class MarrDock2Unloading(smach.State):
    def __init__(self):
        """
        """
        smach.State.__init__(
            self,
            outcomes=[
                "marr_dock2unloading_done",
                "marr_dock2unloading_succ",
                "marr_dock2unloading_fail",
            ],
            input_keys=["marr_dock2unloading_task_done"],
        )

    def execute(self, ud):
        """
        """
        rospy.logdebug("MD2U - task done: %s" % ud.marr_dock2unloading_task_done)
        start_time = timer()
        end_time = timer()
        timeout = 2

        while end_time - start_time < timeout:
            end_time = timer()
            rospy.logdebug("MarrDock2Unloading - working")
            rospy.sleep(0.22543)

        if end_time % 100 == 0:
            rospy.logdebug("MarrDock2Unloading - fail")
            return "marr_dock2unloading_fail"

        if ud.marr_dock2unloading_task_done:
            rospy.logdebug("MarrDock2Unloading - done")
            return "marr_dock2unloading_done"
        else:
            rospy.logdebug("MarrDock2Unloading - succ")
            return "marr_dock2unloading_succ"


def main():
    rospy.init_node("smach_hw4", log_level=rospy.DEBUG)

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(
        outcomes=["ok", "pre_load_failed", "load_fail", "return_dock2unloading_fail"],
    )
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
                    "MARRUNLOADING2DOCK": "marr_unloading2dock_succ",
                }
            },
            output_keys=["marr_unloading2dock_dock", "marr_unloading2dock_xy_yaw"],
        )

        # Open the container
        with sm_con_pre_loading:
            # Add STATES to the container
            smach.Concurrence.add("UR5INITIALSETUP", UR5InitialSetup())
            smach.Concurrence.add(
                "MARRUNLOADING2DOCK",
                MarrUnloading2Dock(),
            )

        # add the concurrent state machine as state in the top machine
        smach.StateMachine.add(
            "PRE_LOADING",
            sm_con_pre_loading,
            transitions={
                "con1_succ": "UR5LOADOBJECTS",
                "con1_fail": "pre_load_failed",
            },
            remapping={
                "marr_unloading2dock_dock": "top_dock",
                "marr_unloading2dock_xy_yaw": "top_xy_yaw",
            },
        )

        #  add the state to load the marrtino
        smach.StateMachine.add(
            "UR5LOADOBJECTS",
            UR5LoadObjects(),
            transitions={
                "ur5_load_succ": "MARRDOCK2UNLOADING",
                "ur5_load_fail": "load_fail",
            },
            remapping={
                "ur5_load_task_name": "top_task_name",
                "ur5_load_dock": "top_dock",
                "ur5_load_xy_yaw": "top_xy_yaw",
                "ur5_load_task_done": "top_task_done",
            },
        )

        # add state to marrtino go home
        smach.StateMachine.add(
            "MARRDOCK2UNLOADING",
            MarrDock2Unloading(),
            transitions={
                "marr_dock2unloading_done": "ok",
                "marr_dock2unloading_succ": "PRE_LOADING",
                "marr_dock2unloading_fail": "return_dock2unloading_fail",
            },
            remapping={"marr_dock2unloading_task_done": "top_task_done"},
        )

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == "__main__":
    main()
