#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_wrench_keyboard')
import rospy

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped

import sys, select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


forceMsg = Wrench

msg = """
Reading from the keyboard  and Publishing to force!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max force/torque by 10%
w/x : increase/decrease only forces by 10%
e/c : increase/decrease only torque by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

forceBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('Wrench', forceMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.force = 0.0
        self.torque = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, force, torque):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.force = force
        self.torque = torque
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        force_msg = forceMsg()

        if stamped:
            force = force_msg.force
            force_msg.header.stamp = rospy.Time.now()
            force_msg.header.frame_id = force_frame
        else:
            force = force_msg
        while not self.done:
            if stamped:
                force_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into force message.
            force.force.x = self.x * self.force
            force.force.y = self.y * self.force
            force.force.z = self.z * self.force
            force.torque.x = 0
            force.torque.y = 0
            force.torque.z = self.th * self.torque

            self.condition.release()

            # Publish.
            self.publisher.publish(force_msg)

        # Publish stop message when thread exits.
        force.force.x = 0
        force.force.y = 0
        force.force.z = 0
        force.torque.x = 0
        force.torque.y = 0
        force.torque.z = 0
        self.publisher.publish(force_msg)


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(force, torque):
    return "currently:\tforce %s\ttorque %s " % (force,torque)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_wrench_keyboard')

    force = rospy.get_param("~force", 0.1)
    torque = rospy.get_param("~torque", 0.1)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    stamped = rospy.get_param("~stamped", False)
    force_frame = rospy.get_param("~frame_id", '')
    if stamped:
        forceMsg = WrenchStamped
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, force, torque)

        print(msg)
        print(vels(force,torque))
        while(1):
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in forceBindings.keys():
                force = force * forceBindings[key][0]
                torque = torque * forceBindings[key][1]

                print(vels(force,torque))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, force, torque)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

