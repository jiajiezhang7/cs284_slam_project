#!/usr/bin/env python

import time
import rospy
from rospy.exceptions import ROSException

def wait_for_service(service, timeout=30):
    try:
        rospy.wait_for_service(service, timeout)
        return True
    except ROSException:
        return False

if __name__ == '__main__':
    rospy.init_node('delay_start', anonymous=True)
    rospy.loginfo("Delaying start by 5 seconds...")
    time.sleep(7)
    
    services = ['/gazebo/set_physics_properties', '/gazebo/spawn_sdf_model']
    for service in services:
        if wait_for_service(service):
            rospy.loginfo(f"Service {service} is now available.")
        else:
            rospy.logwarn(f"Service {service} is not available. Exiting.")
            exit(1)
    
    rospy.loginfo("Delay complete.")
