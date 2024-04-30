import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


initial_learning_rate = 0.1

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
    print("skeleton\n", skeleton)
    print("first chain\n", chains[0])
    print("TEST")
    IK.set_angle(B3.idx, 90, skeleton)
    IK.update_skeleton(skeleton)
    print(IK.get_joint_angles(skeleton))
    jacobian = IK.compute_jacobian(chains, skeleton)
    print("jacobian\n", jacobian)
    hessian = IK.compute_hessian(chains, skeleton, jacobian)
    print("hessian\n", hessian)
    print(len(hessian))
    gradient = IK.compute_gradient(chains, skeleton, jacobian)
    print("gradient \n", gradient)
    objective = IK.compute_objective(chains, skeleton)
    print("objective \n", objective)
    end_effector = IK.get_end_effector(chains[0], skeleton)
    print(end_effector)


    print(inverse_kinematics(skeleton, chains, [0,0,1], max_iterations=100, tolerance=0.01))

def inverse_kinematics(skeleton, chains, target_pos, max_iterations=100, tolerance=0.01):
    learning_rate = initial_learning_rate
    joint_angles = IK.get_joint_angles(skeleton)  # Initialize joint_angles before the loop

    for _ in range(max_iterations):
        IK.update_skeleton(skeleton)
        current_pos = IK.get_end_effector(chains[0], skeleton)
        error = np.linalg.norm(target_pos - current_pos)
        if error < tolerance:
            print("Convergence achieved with error:", error)
            break

        jacobian = IK.compute_jacobian(chains, skeleton)
        gradient = IK.compute_gradient(chains, skeleton, jacobian)

        if jacobian.shape[1] != 3 or gradient.shape[0] != 3:
            print("Jacobian or gradient incorrectly shaped", jacobian.shape, gradient.shape)
            break

        adjustment = np.dot(np.linalg.pinv(jacobian), gradient.reshape(-1, 1)).flatten() * learning_rate
        joint_angles -= adjustment  # Update the joint angles with the adjustment

        IK.set_joint_angles(skeleton, joint_angles)

    return joint_angles


    return joint_angles




if __name__ == '__main__':
    show_simple_setup()
