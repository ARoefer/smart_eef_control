import rospy
import numpy as np
import symengine as sp
import math

from control_msgs.msg import GripperCommandActionGoal as GCAGMsg

from giskardpy.symengine_robot import Robot
from giskardpy.symengine_controller import SymEngineController
from giskardpy.symengine_wrappers import pos_of, rot_of, axis_angle_from_matrix, rotation3_axis_angle, rpy_from_matrix, vector3, eye, diag, rotation3_rpy
from giskardpy.input_system import Vector3Input, RPYInput
from giskardpy.qp_problem_builder import SoftConstraint as SC

from smart_eef_control.msg import RemoteControl as RemoteControlMsg
from smart_eef_control.msg import DeviceMotion  as DeviceMotionMsg
from smart_eef_control.srv import *
from smart_eef_control.bc_controller_wrapper import BCControllerWrapper
from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import Vector3Stamped as Vector3SMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg
from visualization_msgs.msg import Marker as MarkerMsg

rad2deg = 57.2957795131
deg2rad = 1.0 / rad2deg

def symbol_formatter(symbol_name):
    if '__' in symbol_name:
        raise Exception('Illegal character sequence in symbol name "{}"! Double underscore "__" is a separator sequence.'.format(symbol_name))
    return sp.Symbol(symbol_name.replace('/', '__'))


# def rotation3_rpy(r, p, y):
#     """ Conversion of roll, pitch, yaw to 4x4 rotation matrix according to:
#         https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
#     """
#     # TODO don't split this into 3 matrices

#     return np.array([[np.cos(p)*np.cos(y), -np.sin(y)*np.cos(r) + np.sin(p)*np.sin(r)*np.cos(y), np.sin(r)*np.sin(y) + np.sin(p)*np.cos(r)*np.cos(y)],
#             [np.sin(y)*np.cos(p), np.cos(r)*np.cos(y) + np.sin(p)*np.sin(r)*np.sin(y), -np.sin(r)*np.cos(y) + np.sin(p)*np.sin(y)*np.cos(r)],
#             [-np.sin(p), np.sin(r)*np.cos(p), np.cos(r)*np.cos(p)]])

def rpy_from_matrix(matrix):
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    r = math.atan2(matrix[2, 1], matrix[2, 2])
    p = math.atan2(-matrix[2, 0], sy)
    y = math.atan2(matrix[1, 0], matrix[0, 0])

    return r, p, y
    

class Endeffector(object):
    def __init__(self, robot, link, reference_frame, vel_limit=0.6, symmetry=None, gripper_topic=None):
        self.link = link
        self.fk_frame = robot.get_fk_expression(reference_frame, link)
        self.vel_limit = vel_limit
        self.pub_gcmd = rospy.Publisher(gripper_topic, GCAGMsg, queue_size=1, tcp_nodelay=True) if gripper_topic != None else None
        self.last_command = None
        self.current_rpy  = vector3(0,0,0)
        self.current_lin_vel = vector3(0,0,0)
        self.global_command  = False 
        self.symmetry = symmetry
        self.command_transform = eye(4)
        self.controller_transform = eye(4)

    def generate_constraints(self):
        self.input_lin_vel   = Vector3Input.prefix(symbol_formatter, '{}_lin_vel'.format(self.link))
        self.input_rot_goal  = RPYInput.prefix_constructor(symbol_formatter, '{}_rot_goal'.format(self.link))
        self.input_rot_trans = RPYInput.prefix_constructor(symbol_formatter, '{}_rot_trans'.format(self.link))
        self.input_rot_offset = RPYInput.prefix_constructor(symbol_formatter, '{}_rot_offset'.format(self.link))

        soft_constraints = {'{}_vel_{}'.format(self.link, x): SC(self.input_lin_vel.get_expression()[x],
                                                       self.input_lin_vel.get_expression()[x],
                                                       1,
                                                       pos_of(self.fk_frame)[x]) for x in range(3)}

        self.goal_rotation = self.input_rot_offset.get_expression() * self.input_rot_goal.get_expression()
                             #self.input_rot_trans.get_expression()  * \
                             
        axis, angle = axis_angle_from_matrix((rot_of(self.fk_frame).T * self.goal_rotation))
        r_rot_control = axis * angle

        hack = rotation3_axis_angle([0, 0, 1], 0.0001)

        axis, angle = axis_angle_from_matrix(rot_of(self.fk_frame) * hack)
        c_aa = (axis * angle)

        soft_constraints['{} align rotation 0'.format(self.link)] = SC(lower=r_rot_control[0],
                                                  upper=r_rot_control[0],
                                                  weight=1,
                                                  expression=c_aa[0])
        soft_constraints['{} align rotation 1'.format(self.link)] = SC(lower=r_rot_control[1],
                                                  upper=r_rot_control[1],
                                                  weight=1,
                                                  expression=c_aa[1])
        soft_constraints['{} align rotation 2'.format(self.link)] = SC(lower=r_rot_control[2],
                                                  upper=r_rot_control[2],
                                                  weight=1,
                                                  expression=c_aa[2])
        return soft_constraints

    def process_input(self, subs, lx, ly, lz, ax, ay, az, oy, scale=1.0, command_type='global'):
        now = rospy.Time.now()

        if self.last_command == None:
            rotation_matrix = self.fk_frame.subs(subs)
            r, p, y = rpy_from_matrix(rotation_matrix)
            subs[self.input_rot_offset.r] = r
            subs[self.input_rot_offset.p] = p
            subs[self.input_rot_offset.y] = y
            
            if self.symmetry == 'xz' and rotation_matrix[2,2] < 0:
                self.command_transform = diag(1, -1, -1, 1)
            else:
                self.command_transform = eye(4)

            if command_type == 'relative':
                self.controller_transform = rotation3_rpy(0,0, oy)
                # r2, p2, y2 = rpy_from_matrix(rotation_matrix.T * self.controller_transform)
                # subs[self.input_rot_trans.r] = r2
                # subs[self.input_rot_trans.p] = p2
                # subs[self.input_rot_trans.y] = y2
            # elif command_type == 'global':
            #     r2, p2, y2 = rpy_from_matrix(rotation_matrix.T)
            #     subs[self.input_rot_trans.r] = r2
            #     subs[self.input_rot_trans.p] = p2
            #     subs[self.input_rot_trans.y] = y2
            else:
                self.controller_transform = eye(4)
                # subs[self.input_rot_trans.r] = 0
                # subs[self.input_rot_trans.p] = 0
                # subs[self.input_rot_trans.y] = 0

        self.last_command    = now
        self.command_type    = command_type
        self.current_rpy     = self.command_transform * vector3(ax, ay, az)
        self.current_lin_vel = self.controller_transform * vector3(lx, ly,  lz) * self.vel_limit * scale

    def update_subs(self, subs):
        if self.last_command != None:
            now = rospy.Time.now()
            deltaT = (now - self.last_command).to_sec()
            rotation = self.fk_frame.subs(subs)
            if deltaT > 0.1:
                self.last_command = None
                self.stop_motion(subs)
                return True

            if self.command_type == 'global' or self.command_type == 'relative':
                transformed_vel = self.current_lin_vel
            else:
                transformed_vel = rotation * self.command_transform * self.current_lin_vel

            subs[self.input_lin_vel.x] = transformed_vel[0]
            subs[self.input_lin_vel.y] = transformed_vel[1]
            subs[self.input_lin_vel.z] = transformed_vel[2]
            subs[self.input_rot_goal.r] = self.current_rpy[0]
            subs[self.input_rot_goal.p] = self.current_rpy[1]
            subs[self.input_rot_goal.y] = self.current_rpy[2]
            return True
        return False

    def stop_motion(self, subs):
        r, p, y = rpy_from_matrix(self.fk_frame.subs(subs))
        subs[self.input_rot_offset.r] = r
        subs[self.input_rot_offset.p] = p
        subs[self.input_rot_offset.y] = y
        # subs[self.input_rot_trans.r] = 0
        # subs[self.input_rot_trans.p] = 0
        # subs[self.input_rot_trans.y] = 0
        self.current_lin_vel = vector3(0,0,0)
        self.current_rpy = vector3(0,0,0)
        subs[self.input_lin_vel.x]  = 0
        subs[self.input_lin_vel.y]  = 0
        subs[self.input_lin_vel.z]  = 0
        subs[self.input_rot_goal.r] = self.current_rpy[0]
        subs[self.input_rot_goal.p] = self.current_rpy[1]
        subs[self.input_rot_goal.y] = self.current_rpy[2]

    @classmethod
    def from_dict(cls, robot, base_frame, init_dict):
        link = init_dict['link']
        vel_limit = init_dict['velocity_limit'] if 'velocity_limit' in init_dict else 0.6
        gripper_topic = init_dict['gripper_topic'] if 'gripper_topic' in init_dict else None
        symmetry = init_dict['symmetry'] if 'symmetry' in init_dict else None
        return cls(robot, link, base_frame, vel_limit, symmetry, gripper_topic)


class ControlNode(object):
    def __init__(self, eefs, robot):
        self.robot = robot
        self.controller = BCControllerWrapper(self.robot)
        self.pub_cmd = rospy.Publisher('commands', JointStateMsg, queue_size=1, tcp_nodelay=True)

        self.eefs = {e.link: e for e in eefs}
        self.global_command = False
        self.current_eef = self.eefs.values()[0]
        self.initialized = False

        self.services = [rospy.Service('get_endeffectors', GetEndeffectors, self.srv_get_endeffectors)]

        soft_constraints = {}
        for e in self.eefs.values():
            soft_constraints.update(e.generate_constraints())

        self.controller.init(soft_constraints)

        self.sub_dm = rospy.Subscriber('/remote_control', RemoteControlMsg, self.cb_device_motion, queue_size=1)
        self.sub_js = rospy.Subscriber('joint_states', JointStateMsg, self.cb_joint_state, queue_size=1)

    def cb_joint_state(self, msg):
        self.controller.set_robot_js({msg.name[x]: msg.position[x] for x in range(len(msg.name))})

        if not self.initialized:
            for e in self.eefs.values():
                e.stop_motion(self.controller.current_subs)
            self.initialized = True

        if self.current_eef.update_subs(self.controller.current_subs):
            cmd = self.controller.get_cmd()
            cmd_msg = JointStateMsg()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.effort = [0] * len(cmd.items())
            cmd_msg.position = [0] * len(cmd.items())
            for j, v in cmd.items():
                cmd_msg.name.append(j)
                cmd_msg.velocity.append(v)
            self.pub_cmd.publish(cmd_msg)

    def cb_device_motion(self, msg):
        if self.initialized:
            if msg.controlled_id != self.current_eef.link:
                self.current_eef.stop_motion(self.controller.current_subs)
                if msg.controlled_id not in self.eefs:
                    raise Exception('Unknown end effector "{}"'.format(msg.controlled_id))
                self.current_eef = self.eefs[msg.controlled_id]

            self.current_eef.process_input(self.controller.current_subs, 
                                           msg.linear_input.x, 
                                           msg.linear_input.y,  
                                           msg.linear_input.z,
                                           msg.angular_input.x * deg2rad, 
                                           msg.angular_input.y * deg2rad, 
                                           msg.angular_input.z * deg2rad,
                                           msg.controller_yaw * deg2rad,
                                           msg.linear_scale,
                                           msg.command_type)

    def stop(self):
        self.sub_dm.unregister()
        self.pub_cmd.unregister()

    def srv_get_endeffectors(self, req):
        res = GetEndeffectorsResponse()
        res.robot = self.robot._urdf_robot.name
        res.endeffectors = self.eefs.keys()
        return res

    @classmethod
    def from_dict(cls, init_dict):
        robot = Robot(rospy.get_param('/robot_description'), 0.6)
        base_frame = init_dict['reference_frame']
        eefs = [Endeffector.from_dict(robot, base_frame, d) for d in init_dict['end_effectors']]
        return cls(eefs, robot)