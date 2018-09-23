import rospy
import os

from giskardpy import print_wrapper
from giskardpy.symengine_controller import SymEngineController
from giskardpy.qp_problem_builder import JointConstraint
from giskardpy.qp_problem_builder import SoftConstraint as SC
from giskardpy.god_map import GodMap
from giskardpy.symengine_wrappers import Symbol

def res_pkg_path(rpath):
    """Resolves a ROS package relative path to a global path.
    :param rpath: Potential ROS URI to resolve.
    :type rpath: str
    :return: Local file system path
    :rtype: str
    """
    if rpath[:10] == 'package://':
        paths = os.environ['ROS_PACKAGE_PATH'].split(':')

        rpath = rpath[10:]
        pkg = rpath[:rpath.find('/')]

        for rpp in paths:
            if rpp[rpp.rfind('/') + 1:] == pkg:
                return '{}/{}'.format(rpp[:rpp.rfind('/')], rpath)
            if os.path.isdir('{}/{}'.format(rpp, pkg)):
                return '{}/{}'.format(rpp, rpath)
        raise Exception('Package "{}" can not be found in ROS_PACKAGE_PATH!'.format(pkg))
    return rpath


class BCControllerWrapper(SymEngineController):
    def __init__(self, robot, print_fn=print_wrapper):
        self.path_to_functions = res_pkg_path('package://smart_eef_control/.controllers/')
        self.controlled_joints = []
        self.hard_constraints = {}
        self.joint_constraints = {}
        self.qp_problem_builder = None
        self.robot = robot
        self.current_subs = {}
        self.print_fn = print_fn

    def init(self, soft_constraints):
        free_symbols = set()
        for sc in soft_constraints.values():
            for f in sc:
                if hasattr(f, 'free_symbols'):
                    free_symbols = free_symbols.union(f.free_symbols)
        super(BCControllerWrapper, self).set_controlled_joints([j for j in self.robot.get_joint_names() if self.robot.joint_states_input.joint_map[j] in free_symbols])
        if hasattr(self.robot, 'soft_dynamics_constraints'):
            for k, c in self.robot.soft_dynamics_constraints.items():
                if len(c.expression.free_symbols.intersection(free_symbols)) > 0:
                    soft_constraints[k] = c
                    for f in c:
                        if hasattr(f, 'free_symbols'):
                            free_symbols = free_symbols.union(f.free_symbols)

        for jc in self.joint_constraints.values():
            for f in jc:
                if hasattr(f, 'free_symbols'):
                    free_symbols = free_symbols.union(f.free_symbols)
        for hc in self.hard_constraints.values():
            for f in hc:
                if hasattr(f, 'free_symbols'):
                    free_symbols = free_symbols.union(f.free_symbols)
        #print('  \n'.join([str(s) for s in free_symbols]))
        self.free_symbols = free_symbols

        super(BCControllerWrapper, self).init(soft_constraints, free_symbols, self.print_fn)

    def set_robot_js(self, js):
        for j, s in js.items():
            if j in self.robot.joint_states_input.joint_map:
                self.current_subs[self.robot.joint_states_input.joint_map[j]] = s

    def get_cmd(self, nWSR=None):
        return super(BCControllerWrapper, self).get_cmd({str(s): p for s, p in self.current_subs.items()}, nWSR)

    def stop(self):
        pass