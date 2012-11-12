#!/usr/bin/env python

import roslib; roslib.load_manifest('spacenav_telemanip')

import rospy
import tf
import telemanip_msgs.msg as telemanip_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import math
import PyKDL
import copy
import threading

def multiply_quat(q0,q1):
    [x0,y0,z0,w0] = q0
    [x1,y1,z1,w1] = q1
    return ( (w0*x1 + x0*w1 + y0*z1 - z0*y1),
             (w0*y1 - x0*z1 + y0*w1 + z0*x1),
             (w0*z1 + x0*y1 - y0*x1 + z0*w1),
             (w0*w1 - x0*x1 - y0*y1 - z0*z1) )

class TwistFrameIntegrator(object):
    def __init__(self):
        # Get the node params
        self.linear_multiplier = rospy.get_param('~linear_multiplier')
        self.angular_multiplier = rospy.get_param('~angular_multiplier')
        self.broadcast_rate = rospy.get_param('~broadcast_rate')
        self.body_fixed = rospy.get_param('~body_fixed')

        # Initialize the frame we're going to publish
        self.transform = PyKDL.Frame()
        self.transform_out = PyKDL.Frame()
        self.translation = (0,0,0)
        self.rotation = (0,0,0,1)
        self.time = rospy.Time.now()
        self.frame_id = rospy.get_param('~frame_id')
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.telemanip_command = telemanip_msgs.TelemanipCommand()

        # Telemani command publisher
        self.telemanip_pub = rospy.Publisher('telemanip_command', telemanip_msgs.TelemanipCommand)

        # Synchronization
        self.broadcast_lock = threading.Lock()

        # Initialze TF structures
        self.broadcaster = tf.TransformBroadcaster()

        # Subscribe to twist inputs
        self.joy_sub = rospy.Subscriber(
                "joy", sensor_msgs.Joy,
                self.joy_cb)

        # Broadcast the frame at a given rate
        self.broadcast_thread = threading.Thread(target=self.broadcast_loop)
        self.broadcast_thread.start()

    def broadcast_loop(self):
        """
        Broadcast the integrated TF frame at a fixed rate.
        """
        rate = rospy.Rate(self.broadcast_rate)
        while not rospy.is_shutdown():
            with self.broadcast_lock:
                try:
                    # Broadcast the frame
                    self.broadcaster.sendTransform(
                            (self.transform_out.p.x(), self.transform_out.p.y(), self.transform_out.p.z()),
                            self.transform_out.M.GetQuaternion(),
                            rospy.Time.now(),
                            self.child_frame_id,
                            self.frame_id)
                except Exception as ex:
                    rospy.logerr("Failed to broadcast transform: "+str(ex))
            # Don't spam TF
            rate.sleep()

    def joy_cb(self,msg):
        """
        This callback integrates the linear and angular velocities into a TF
        frame, and then broadcasts the frame.
        """
        with self.broadcast_lock:
            # Convert angular rotation vector to quaternion
            q_velocity = (
                    msg.axes[3] * math.sin(self.angular_multiplier/2),
                    msg.axes[4] * math.sin(self.angular_multiplier/2),
                    msg.axes[5] * math.sin(self.angular_multiplier/2),
                    math.cos(self.angular_multiplier/2))

            q_velocity_norm = math.sqrt(math.pow(q_velocity[0],2) + math.pow(q_velocity[1],2) + math.pow(q_velocity[2],2) + math.pow(q_velocity[3],2))
            q_velocity = [i/q_velocity_norm for i in q_velocity]

            if self.body_fixed:
                # Integrate linear velocity in local frame
                self.transform.p += self.transform.M*(self.linear_multiplier*PyKDL.Vector(msg.axes[0],msg.axes[1],msg.axes[2]))
                # Integrate angular velocity in local frame
                self.rotation = multiply_quat(self.rotation,q_velocity)
                self.transform.M = PyKDL.Rotation.Quaternion(*self.rotation)
                
                # Copy this transform
                self.transform_out = PyKDL.Frame(self.transform)
            else:
                # Integrate linear velocity
                self.transform.p += self.linear_multiplier*PyKDL.Vector(msg.axes[0],msg.axes[1],msg.axes[2])
                # Integrate angular velocity
                self.rotation = multiply_quat(q_velocity,self.rotation)
                self.transform.M = PyKDL.Rotation.Quaternion(*self.rotation)

                # Invert the transform to get parent-frame relative transform
                self.transform_out = self.transform

            self.transform_out.M.DoRotZ(-math.pi/2)
            self.transform_out.M.DoRotX(-math.pi/2)


        self.telemanip_command.header = msg.header
        self.telemanip_command.header.frame_id = self.frame_id
        self.telemanip_command.posetwist.pose.position.x = self.transform_out.p.x()
        self.telemanip_command.posetwist.pose.position.y = self.transform_out.p.y()
        self.telemanip_command.posetwist.pose.position.z = self.transform_out.p.z()

        (x,y,z,w) = self.transform_out.M.GetQuaternion()
        self.telemanip_command.posetwist.pose.orientation.x = x
        self.telemanip_command.posetwist.pose.orientation.y = y
        self.telemanip_command.posetwist.pose.orientation.z = z
        self.telemanip_command.posetwist.pose.orientation.w = w

        self.telemanip_command.resync_pose = msg.buttons[1]

        self.telemanip_command.grasp_opening = 1 - msg.buttons[0]
        self.telemanip_command.deadman_engaged = True
        self.telemanip_command.estop = False

        self.telemanip_pub.publish(self.telemanip_command)

def main():
    rospy.init_node('spacenav_telemanip')
    
    twist_frame_integrator = TwistFrameIntegrator()

    rospy.loginfo("Spinning...")
    rospy.spin()
    rospy.loginfo("Done.")


if __name__ == '__main__':
    main()
