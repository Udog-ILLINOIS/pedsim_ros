#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from pedsim_agents.states import PedsimStates
import rospy
import std_msgs.msg

from pedsim_agents.config import Topics
from pedsim_agents.utils import NList, InMsg, InData, WorkData, OutMsg

from pedsim_agents.semantic import SemanticProcessor
from pedsim_agents.forces.forces import Forcemodels

# Main function.
def main():
    rospy.init_node("pedsim_agents")

    #TODO automate this
    Topics.INPUT = os.path.join(rospy.get_name(), Topics.INPUT)
    Topics.FEEDBACK = os.path.join(rospy.get_name(), Topics.FEEDBACK)
    Topics.SEMANTIC = os.path.join(rospy.get_name(), Topics.SEMANTIC)

    states = PedsimStates()
    forcemodel = Forcemodels(str(rospy.get_param("~forcemodel", "")))
    semantic = SemanticProcessor()

    pub = rospy.Publisher(
        name=Topics.FEEDBACK,
        data_class=OutMsg,
        queue_size=1
    )

    def callback(input_msg: InMsg):

        if not running:
            return;

        input_data = InData(
            header=input_msg.header,
            agents=NList(input_msg.agent_states),
            robots=NList(input_msg.robot_states),
            groups=NList(input_msg.simulated_groups),
            waypoints=NList(input_msg.simulated_waypoints),
            walls=NList(input_msg.walls),
            obstacles=NList(input_msg.obstacles)
        )
        work_data = WorkData.construct(input_data)

        states.pre(input_data, work_data)
        forcemodel.run(input_data, work_data)
        states.post(input_data, work_data)

        semantic_data = semantic.calculate(input_data, work_data, states.semantic())

        pub.publish(work_data.msg(input_msg.header))
        semantic.publish(input_msg.header.stamp, semantic_data)
    

    while True:

        running: bool = False

        if rospy.get_param("/resetting", False) == True:
            rospy.wait_for_message("/reset_end", std_msgs.msg.Empty)

        states.reset()
        forcemodel.reset()
        semantic.reset()

        sub = rospy.Subscriber(
            name=Topics.INPUT,
            data_class=InMsg,
            callback=callback,
            queue_size=1
        )

        running = True
        
        rospy.wait_for_message("/reset_start", std_msgs.msg.Empty)

        running = False
        sub.unregister()

if __name__ == "__main__":
    main()