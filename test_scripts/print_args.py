#!/usr/bin/env python3

# Testeo de parametros y argumentos en ROS

import rospy
import os

def main():
    rospy.init_node('print_args_node')
    
    nameParams = ['example_param1', 'print_args/example_param2', 'ip', 'port']
    
    for nameParam in nameParams:
        valueParam = rospy.get_param(nameParam, 'Unknown')
        rospy.loginfo(f"Value of '{nameParam}': {valueParam}")
    
    ip = rospy.get_param('ip', 'Error')
    ip = os.path.expandvars(ip)
    rospy.loginfo(f"Expanded value of 'ip': {ip}")

if __name__ == '__main__':
    main()
