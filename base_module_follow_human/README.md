rosrun hsrb_state_checker hsrb_state_checker_node.py

roslaunch follow_human follow_human.launch 

rosrun actionlib axclient.py /follow_human_action

例

avoid_function: 0

x: 1.0

y: 0.0