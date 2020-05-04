import rospy
from std_msgs.msg import Float32
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from random import random
from geometry_msgs.msg import Wrench, WrenchStamped, Vector3 

def get_individual_thruster_publisher(thruster_id='0'):
    return rospy.Publisher('/urab_sub/thrusters/' + thruster_id + '/input', FloatStamped, queue_size=10)
def get_individual_thruster_message(power=1):
    msg = FloatStamped()
    msg.header.stamp = rospy.Time.now()
    msg.data = power
    return msg

def get_thruster_manager_publisher():
    return rospy.Publisher('/urab_sub/thruster_manager/input', Wrench, queue_size=10)
def get_thruster_manager_message(fx=1, fy=1, fz=1, tx=1, ty=1, tz=1):
    force = Vector3()
    force.x = fx
    force.y = fy
    force.z = fz
    torque = Vector3()
    torque.x = tx
    torque.y = ty
    torque.z = tz
    msg = Wrench()
    msg.force = force
    msg.torque = torque
    return msg

MODES = ['thruster_manager', 'thrusters_direct']
NUM_THRUSTERS = 8
mode = 1
if __name__ == "__main__":
    rospy.init_node('test_setter')

    publishers = []
    if MODES[mode] == 'thruster_manager':
        publishers.append(rospy.Publisher('/urab_sub/thruster_manager/input', Wrench, queue_size=10))
    elif MODES[mode] == 'thrusters_direct':
        for thruster_id in range(NUM_THRUSTERS):
            publishers.append(rospy.Publisher('/urab_sub/thrusters/{}/input'.format(thruster_id), FloatStamped, queue_size=10))

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        messages = []
        # msg = get_individual_thruster_message(20)
        if MODES[mode] == 'thruster_manager':
            messages.append(get_thruster_manager_message(1000000000,0,0,1000000000,0,0))
        elif MODES[mode] == 'thrusters_direct':
            for __ in range(NUM_THRUSTERS):
                msg = FloatStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = 10
                messages.append(msg)

        for i in range(len(publishers)):
            publishers[i].publish(messages[i])
        print('messages', messages)
        rate.sleep()
