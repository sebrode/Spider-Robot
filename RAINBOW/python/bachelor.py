import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def show_simple_setup():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(
        0), beta=0.0, gamma=0.0, tx=26.8, ty=19.2, tz=28.5)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(
        0), beta=0.0, gamma=0.0, tx=78.0, ty=30.0, tz=40.5)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(
        0), beta=0.0, gamma=0.0, tx=83.0, ty=41.466, tz=10.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=IK.degrees_to_radians(
        0), beta=0.0, gamma=0.0, tx=82.0, ty=30.0, tz=16.5)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)
    # print(skeleton)
    print(chains[0])
    # print("TEST")
    IK.set_angle(B3.idx, 90, skeleton)
    IK.update_skeleton(skeleton)
    print(IK.get_joint_angles(skeleton))
    jacobian = IK.compute_jacobian(chains, skeleton)
    print(jacobian)
    hessian = IK.compute_hessian(chains, skeleton, jacobian)
    print(hessian)
    # print(len(hessian))
    gradient = IK.compute_gradient(chains, skeleton, jacobian)
    print(gradient)
    objective = IK.compute_objective(chains, skeleton)
    print(objective)
    # end_effector = IK.get_end_effector(chains, skeleton)
    # print(end_effector)


if __name__ == '__main__':
    show_simple_setup()
