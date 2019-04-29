#!/usr/bin/env python
from __future__ import print_function
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import actionlib_tutorials.msg
from std_msgs.msg import Bool

import actionlib
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal

def action_client():
    client = actionlib.SimpleActionClient('depth_hold_action_server', DepthHoldAction)
    client.wait_for_server()
    goal = DepthHoldGoal(depth = 2)
    client.send_goal(goal)
    ready = client.get_state()
    #client.wait_for_result()
    return client

def mission_trigger_callback(trigger_signal):
    print('Received signal')
    global mission_in_progress
    if(trigger_signal.data == True):
        mission_in_progress = not mission_in_progress
        print(mission_in_progress)

def request_preempt():
    global mission_in_progress
    if mission_in_progress:
        return False
    else:
        return True

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['doing','waiting'])
        # subscribe to signal
        print('Init')
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        #print('Executing')
        if mission_in_progress == False:
            #print('Waiting')
            self.rate.sleep()
            return 'waiting'
        else:
            print('Jumping')
            return 'doing'

class Dive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['submerged','going','preempted'])
        print('Diving')
        self.client = action_client()


    def execute(self, userdata):
        print('diving')
        # turn on diving stuff and do that shit
        if request_preempt():
            return 'preempted'

        if False:#self.client.get_state[1]:
            return 'submerged'
        else:
            return 'going'


class Cancel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canceld'])

    def execute(self, userdata):
        #Kill all nodes
        return 'canceld'

'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_heading'])
        result = action_client(order = 20)

    def execute(self):
        if gate_found:
            return 'found'
'''
'''
class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Centered','Lost'])

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)
        result = action_client(order = 20)

    def execute(self):
        rospy.loginfo('Centering')
        if sub_gate == 1:
            return 'Centered'
'''
def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Qualification_run')

    global mission_in_progress
    mission_in_progress = False

    rospy.Subscriber("mission_trigger", Bool, mission_trigger_callback)

    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()


    with sm:
        smach.StateMachine.add('Idle', Idle(),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Dive', Dive(),
                                transitions={'submerged':'Done', 'going':'Dive','preempted':'Cancel'})
        smach.StateMachine.add('Cancel', Cancel(),
                                transitions={'canceld':'Idle'})

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
