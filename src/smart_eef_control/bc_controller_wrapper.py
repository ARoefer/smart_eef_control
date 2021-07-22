import os

import kineverse.gradients.gradient_math as gm
from kineverse.model.geometry_model       import GeometryModel
from kineverse.motion.min_qp_builder      import MinimalQPBuilder, \
                                                 generate_controlled_values, \
                                                 depth_weight_controlled_values

class BCControllerWrapper(object):
    def __init__(self, km, robot_path, soft_constraints):
        robot = km.get_data(robot_path)
        robot_joints = {j.position for j in robot.joints.values() if hasattr(j, 'position')}
        relevant_symbols   = robot_joints.intersection(sum([list(gm.free_symbols(c.expr)) for c in soft_constraints.values()], []))
        controlled_symbols = {gm.DiffSymbol(s) for s in relevant_symbols}

        constraints = km.get_constraints_by_symbols(relevant_symbols.union(controlled_symbols))
        controlled_values, constraints = generate_controlled_values(constraints, controlled_symbols)

        if isinstance(km, GeometryModel):
            controlled_values = depth_weight_controlled_values(km, controlled_values, exp_factor=1.02)

        self.qpb = MinimalQPBuilder(constraints, soft_constraints, controlled_values)
        
        self.joint_mapping = {jn: j.position for jn, j in robot.joints.items() if hasattr(j, 'position')}
        self.cmd_mapping   = {gm.DiffSymbol(jp): jn for jn, jp in self.joint_mapping.items()}
        self.current_subs  = {}

    def set_robot_js(self, js):
        for j, s in js.items():
            if j in self.joint_mapping:
                self.current_subs[self.joint_mapping[j]] = s

    def get_cmd(self, nWSR=None):
        return self.qpb.get_cmd(self.current_subs, nWSR)

    def num_joints(self):
        return len(self.cmd_mapping)
