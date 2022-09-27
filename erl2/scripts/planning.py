#! /usr/bin/env python

## @package erl2
#
# \file planning.py
# \brief This node is used for replanning when the plan does not have success
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 27/09/2022
# 
# \details
#
#  Client: <BR>
#	/rosplan_problem_interface/problem_generation_server	
#
#	/rosplan_planner_interface/planning_server
#
#	/rosplan_parsing_interface/parse_plan
#
#	/rosplan_plan_dispatcher/dispatch_plan
#
#	/rosplan_knowledge_base/update
#
#  Description: <BR>
#	This node is used for replanning and start again the behaviour hen the plan does not have success.
#	It updates the knowledge base to go restore the initial condition, and after that the plan is generated and dispatched
#	again.
#

import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time

##
#	Function for calling the services needed to start the plan
#
def init_plan():
    global problem_generation, planning, parsing, dispatch, update
    print("\nINSIDE INIT_PLAN\n")
    
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    
    print("\nALL SERVERS STARTED\n")

##
#	Function to update the hint_taken predicate
#
def update_hint_taken(name):
	req=KnowledgeUpdateServiceRequest()
	req.update_type=2
	req.knowledge.is_negative=False
	req.knowledge.knowledge_type=1
	req.knowledge.attribute_name= 'hint_taken'
	req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', name))	
	result=update(req)
    
##
#	Function to update the hypo_complete predicate
#
def update_hypo_complete():
	req=KnowledgeUpdateServiceRequest()
	req.update_type=2
	req.knowledge.is_negative=False
	req.knowledge.knowledge_type=1
	req.knowledge.attribute_name= 'hypo_complete'
	result=update(req)	

##
#	Function to update the hypo_to_check predicate
#
def update_hypo_to_check():
	req=KnowledgeUpdateServiceRequest()
	req.update_type=2
	req.knowledge.is_negative=False
	req.knowledge.knowledge_type=1
	req.knowledge.attribute_name= 'hypo_to_check'
	result=update(req)
    
##
#	Main function of the planning_loop node.
#	It contains a while loop to make the plan start again until the
#	goal is reached. It also calls the function for updating the knowledge base 
#	so that every plan starts from the same point.
#
def main():
	global pub_, active_, act_s
	rospy.init_node('planning_loop')
	init_plan()
	success=False
	goal=False
    # until the feedback from the planner is not true
	while ( success== False or goal == False):
		# generate the problem
		response_pg=problem_generation()
		print('problem generated')
		time.sleep(1)
        # generate the plan
		response_pl=planning()
		print('plan generated')
		time.sleep(1)
        # parse the plan 
		response_pars=parsing()
		print('plan parsed')
		time.sleep(1)
        # read the feedback
		response_dis=dispatch()
		print(response_dis.success)
		print(response_dis.goal_achieved)
		success= response_dis.success
		goal= response_dis.goal_achieved
        # update the knowledge base
		update_hint_taken('wp0')
		update_hint_taken('wp1')
		update_hint_taken('wp2')
		update_hint_taken('wp3')
		update_hypo_complete()
		update_hypo_to_check()
		time.sleep(1)
	print ( 'SUCCESS')

if __name__ == '__main__':
    main()
