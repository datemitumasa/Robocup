# coding: utf-8
import easygui
import json
import os

def set_ip():
    ip_setting = {}
    try:
        ip_setting = json.load( open( "ip_add.txt" , "r" ) )
    except:
        pass

    fields = ["ROS_MASTER_URI", "ROS_IP"]
    values = []

    for f in fields:
        if f in ip_setting:
            values.append( ip_setting[f] )
        else:
            values.append("")

    values = easygui.multenterbox( "Enter ROS_MASTER_URI and ROS_IP" , "ROS" , fields, values )

    if values:
        for k,v in zip(fields,values):
            ip_setting[k] = v

        json.dump( ip_setting, open( "ip_add.txt" ,"w" ) )

        os.environ["ROS_MASTER_URI"] = values[0]
        os.environ["ROS_IP"] = values[1]

        return True

    return False

def main():
    set_ros_ip()

if __name__ == '__main__':
    main()