import rospy
import math

from collections import namedtuple

from control_msgs.msg import GripperCommandActionGoal as GCAGMsg

import kineverse.gradients.gradient_math as gm
from kineverse.model.paths                import Path
from kineverse.model.geometry_model       import GeometryModel
from kineverse.operations.urdf_operations import load_urdf
from kineverse.urdf_fix                   import hacky_urdf_parser_fix, urdf_filler
from kineverse.motion.min_qp_builder      import SoftConstraint as SC

from smart_eef_control.msg import RemoteControl as RemoteControlMsg
from smart_eef_control.srv import *
from smart_eef_control.bc_controller_wrapper import BCControllerWrapper
from sensor_msgs.msg import JointState as JointStateMsg

from urdf_parser_py.urdf import URDF

SymbolVector3  = namedtuple('SymbolVector3', ['vec'] + list('xyz'))
SymbolPose3RPY = namedtuple('SymbolPose3RPY', ['pose'] + list('xyz') + [f'r{x}' for x in 'rpy'])

rad2deg = 57.2957795131
deg2rad = 1.0 / rad2deg

def rpy_from_matrix(matrix):
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    r = math.atan2(matrix[2, 1], matrix[2, 2])
    p = math.atan2(-matrix[2, 0], sy)
    y = math.atan2(matrix[1, 0], matrix[0, 0])

    return r, p, y

def create_symbol_vec3(prefix):
    if not isinstance(prefix, Path):
        prefix = Path(prefix)
    
    symbols = [p.to_symbol() for p in [prefix + (x,) for x in 'xyz']]
    return SymbolVector3(gm.vector3(*symbols), *symbols)

def create_symbol_frame3_rpy(prefix):
    if not isinstance(prefix, Path):
        prefix = Path(prefix)
    
    symbols = [p.to_symbol() for p in ([prefix + (x,) for x in 'xyz'] + [prefix + (f'r{r}',) for r in 'rpy'])]
    return SymbolPose3RPY(gm.frame3_rpy(*symbols[-3:], gm.point3(*symbols[:3])), *symbols)


class Endeffector(object):
    def __init__(self, robot, link, vel_limit=0.6, symmetry=None, gripper_topic=None):
        self.link      = link
        self.fk_frame  = robot.links[link].pose
        self.vel_limit = vel_limit
        self.pub_gcmd  = rospy.Publisher(gripper_topic, GCAGMsg, queue_size=1, tcp_nodelay=True) if gripper_topic != None else None
        self.last_command = None
        self.current_rpy  = gm.vector3(0,0,0)
        self.current_lin_vel = gm.vector3(0,0,0)
        self.global_command  = False
        self.symmetry = symmetry

        self.input_lin_vel  = create_symbol_vec3(f'{self.link}_lin_vel')
        self.input_rot_goal = create_symbol_frame3_rpy(f'{self.link}_rot_goal')
        self.input_iframe   = create_symbol_frame3_rpy(f'{self.link}_iframe')
        self.input_rot_offset = create_symbol_frame3_rpy(f'{self.link}_rot_offset')


    def generate_constraints(self):
        soft_constraints = {f'{self.link}_vel_{x}': SC((gm.dot(self.input_iframe.pose, self.input_lin_vel.vec))[x],
                                                       (gm.dot(self.input_iframe.pose, self.input_lin_vel.vec))[x],
                                                       1,
                                                       gm.pos_of(self.fk_frame)[x]) for x in range(3)}

        self.goal_rotation = gm.dot(self.input_iframe.pose, self.input_rot_goal.pose, self.input_rot_offset.pose)

        axis, angle = gm.axis_angle_from_matrix(gm.dot(gm.rot_of(self.fk_frame).T, self.goal_rotation))
        r_rot_control = axis * angle

        hack = gm.rotation3_axis_angle([0, 0, 1], 0.0001)

        axis, angle = gm.axis_angle_from_matrix(gm.dot(gm.rot_of(self.fk_frame), hack))
        c_aa = (axis * angle)

        soft_constraints[f'{self.link} align rotation 0'] = SC(lower=r_rot_control[0],
                                                  upper=r_rot_control[0],
                                                  weight=1,
                                                  expr=c_aa[0])
        soft_constraints[f'{self.link} align rotation 1'] = SC(lower=r_rot_control[1],
                                                  upper=r_rot_control[1],
                                                  weight=1,
                                                  expr=c_aa[1])
        soft_constraints[f'{self.link} align rotation 2'] = SC(lower=r_rot_control[2],
                                                  upper=r_rot_control[2],
                                                  weight=1,
                                                  expr=c_aa[2])
        return soft_constraints

    def process_input(self, subs, lx, ly, lz, ax, ay, az, oy, scale=1.0, command_type='global'):
        now = rospy.Time.now()

        if self.last_command is None:
            rotation_matrix = gm.subs(self.fk_frame, subs)
            if command_type == 'relative':
                iframe = gm.rotation3_rpy(0,0, oy)
                subs[self.input_iframe.rr] = 0
                subs[self.input_iframe.rp] = 0
                subs[self.input_iframe.ry] = oy
            elif command_type == 'global':
                iframe = gm.eye(4)
                subs[self.input_iframe.rr] = 0
                subs[self.input_iframe.rp] = 0
                subs[self.input_iframe.ry] = 0
            else:
                iframe = rotation_matrix
                if self.symmetry == 'xz' and rotation_matrix[2,2] < 0:
                    iframe = gm.rotation3_rpy(math.pi, 0, 0) * iframe
                r, p, y = rpy_from_matrix(iframe)
                subs[self.input_iframe.rr] = r
                subs[self.input_iframe.rp] = p
                subs[self.input_iframe.ry] = y

            r, p, y = rpy_from_matrix(iframe.T * rotation_matrix)
            subs[self.input_rot_offset.rr] = r
            subs[self.input_rot_offset.rp] = p
            subs[self.input_rot_offset.ry] = y

        self.last_command    = now
        self.command_type    = command_type
        self.current_rpy     = gm.vector3(ax, ay, az)
        self.current_lin_vel = gm.vector3(lx, ly,  lz) * self.vel_limit * scale
        subs[self.input_lin_vel.x] = self.current_lin_vel[0]
        subs[self.input_lin_vel.y] = self.current_lin_vel[1]
        subs[self.input_lin_vel.z] = self.current_lin_vel[2]
        subs[self.input_rot_goal.rr] = self.current_rpy[0]
        subs[self.input_rot_goal.rp] = self.current_rpy[1]
        subs[self.input_rot_goal.ry] = self.current_rpy[2]

    def update_subs(self, subs):
        if self.last_command != None:
            now = rospy.Time.now()
            deltaT = (now - self.last_command).to_sec()
            if deltaT > 0.1:
                self.last_command = None
                self.stop_motion(subs)
                return True

            #iframe = self.input_iframe.subs(subs)
            return True
        return False

    def stop_motion(self, subs):
        r, p, y = rpy_from_matrix(gm.subs(self.fk_frame, subs))
        subs[self.input_rot_offset.rr] = r
        subs[self.input_rot_offset.rp] = p
        subs[self.input_rot_offset.ry] = y
        subs[self.input_iframe.rr] = 0
        subs[self.input_iframe.rp] = 0
        subs[self.input_iframe.ry] = 0
        self.current_lin_vel = gm.vector3(0,0,0)
        self.current_rpy = gm.vector3(0,0,0)
        subs[self.input_lin_vel.x]  = 0
        subs[self.input_lin_vel.y]  = 0
        subs[self.input_lin_vel.z]  = 0
        subs[self.input_rot_goal.rr] = self.current_rpy[0]
        subs[self.input_rot_goal.rp] = self.current_rpy[1]
        subs[self.input_rot_goal.ry] = self.current_rpy[2]

    @classmethod
    def from_dict(cls, robot, base_frame, init_dict):
        link          = init_dict['link']
        vel_limit     = init_dict['velocity_limit'] if 'velocity_limit' in init_dict else 0.6
        gripper_topic = init_dict['gripper_topic'] if 'gripper_topic' in init_dict else None
        symmetry      = init_dict['symmetry'] if 'symmetry' in init_dict else None
        return cls(robot, link, vel_limit, symmetry, gripper_topic)


class ControlNode(object):
    def __init__(self, km, robot_path, eefs):
        self.km = km
        self.pub_cmd = rospy.Publisher('commands', JointStateMsg, queue_size=1, tcp_nodelay=True)
        self.robot_name = str(robot_path)

        self.eefs = {e.link: e for e in eefs}
        self.global_command = False
        self.current_eef = next(iter(self.eefs.values()))
        self.initialized = False

        self.services = [rospy.Service('get_endeffectors', GetEndeffectors, self.srv_get_endeffectors)]

        soft_constraints = {}
        for e in self.eefs.values():
            soft_constraints.update(e.generate_constraints())

        self.controller = BCControllerWrapper(km, robot_path, soft_constraints)

        self.sub_dm = rospy.Subscriber('/remote_control', RemoteControlMsg, self.cb_device_motion, queue_size=1)
        self.sub_js = rospy.Subscriber('joint_states', JointStateMsg, self.cb_joint_state, queue_size=1)

        self.cmd_msg = JointStateMsg()

    def cb_joint_state(self, msg):
        self.controller.set_robot_js(dict(zip(msg.name, msg.position)))

        if not self.initialized:
            for e in self.eefs.values():
                e.stop_motion(self.controller.current_subs)
            self.initialized = True

        if self.current_eef.update_subs(self.controller.current_subs):
            cmd = self.controller.get_cmd()
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.effort = [0] * len(cmd)
            self.cmd_msg.position = [0] * len(cmd)
            
            self.cmd_msg.name, self.cmd_msg.velocity = list(zip(*cmd.items()))

            self.pub_cmd.publish(self.cmd_msg)

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
        res.robot = self.robot_name
        res.endeffectors = self.eefs.keys()
        return res

    @classmethod
    def from_dict(cls, init_dict):
        km = GeometryModel()
        
        urdf = urdf_filler(URDF.from_xml_string(rospy.get_param('/robot_description')))

        load_urdf(km, Path(urdf.name), urdf)
        km.clean_structure()
        km.dispatch_events()
        base_frame = init_dict['reference_frame']
        eefs = [Endeffector.from_dict(km.get_data(Path(urdf.name)), base_frame, d) for d in init_dict['links']]
        return cls(km, Path(urdf.name), eefs)