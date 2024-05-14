import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np

from scipy.optimize import minimize

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#


def show_simple_setup():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=0.0, tx=0.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=0.0, tx=4.7, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=0.0, tx=6.64, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=0.0, tx=12.6530, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)

    jacobian = IK.compute_jacobian(chains, skeleton)
    print("Jacobian:\n", jacobian)
    hessian = IK.compute_hessian(chains, skeleton, jacobian)
    print("\nHessian:", hessian)
    gradient = IK.compute_gradient(chains, skeleton, jacobian)
    print("Gradient:", gradient)


    def compute_jac(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_jacobian(chains, skeleton)

    def compute_hess(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_hessian(chains, skeleton, IK.compute_jacobian(chains, skeleton))

    def compute_grad(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_gradient(
            chains, skeleton, compute_jac(x, chains, skeleton))

    def compute_obj(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_objective(chains, skeleton)

    # IK.set_angle(B3.idx, 90, skeleton)
    # print(IK.get_joint_angles(skeleton))
    # result = minimize(
    #     fun=compute_obj,
    #     x0=IK.get_joint_angles(skeleton),
    #     method='Newton-CG',
    #     jac=compute_grad,
    #     hess=compute_hess,
    #     args=(chains, skeleton)
    # )

    # print("Optimal solution:", result.x)
    # print("Optimal value:", result.fun)

    def constraint_1(x):
        return [x[3]]

    footPositionMatrix = np.array([
        V3.make(12.0, 10.0, 0.0),
        V3.make(12.0, 11.0, 1.0)
    ])

    def simulateSpider(positionMatrix):
        totalAngleMatrix = np.zeros(
            (len(IK.get_joint_angles(skeleton)), len(positionMatrix)))
        totalObjectMatrix = np.zeros(
            (len(positionMatrix), 1))

        for i in range(len(positionMatrix)):
            chains[0].goal = positionMatrix[i].T
            IK.update_skeleton(skeleton)
            print(IK.compute_jacobian(chains, skeleton))
            print(IK.compute_hessian(chains, skeleton, IK.compute_jacobian(chains, skeleton)))
            print(IK.compute_gradient(chains, skeleton, IK.compute_jacobian(chains, skeleton)))
            input()
            result = minimize(
                fun=compute_obj,
                x0=IK.get_joint_angles(skeleton),
                #method='BFGS',
                method='Newton-CG',
                jac=compute_grad,
                hess=compute_hess,
                args=(chains, skeleton),
                bounds=((-np.pi/2,np.pi/2),(-np.pi/2,np.pi/2),(-np.pi, 0),(-np.pi,np.pi)),
                options={'maxiter': 1000, 'disp': True, 'gtol': 0.001}
                # constraints={'type': 'eq', 'fun': constraint_1}
            )
            print(result)
            totalAngleMatrix[:, i] = result.x
            totalObjectMatrix[i, :] = result.fun

            print("Optimal solution:", result.x)
            print("Optimal value:", result.fun)
        return totalAngleMatrix, totalObjectMatrix

    angleMatrix, objectMatrix = simulateSpider(footPositionMatrix)
    # print(angleMatrix)
    # print(objectMatrix)
    x = [0.0, angleMatrix[0, -1], 0.0, angleMatrix[1, -1], 0.0, 0.0, angleMatrix[2, -1], 0.0, 0.0, angleMatrix[3, -1], 0.0, 0.0]
    print(x)
    print(np.array2string(angleMatrix, separator=', '))
    print(np.array2string(objectMatrix, separator=', '))

    jacobian = IK.compute_jacobian(chains, skeleton)
    # print("Jacobian:\n", jacobian)
    hessian = IK.compute_hessian(chains, skeleton, jacobian)
    # print("\nHessian:", hessian)
    gradient = IK.compute_gradient(chains, skeleton, jacobian)
    print("Gradient:", gradient)
    # objective = IK.compute_objective(chains, skeleton)
    # print("Objective: ", objective)
    # end_effector = IK.get_end_effector(chains[0], skeleton)
    # print(end_effector)
    jacobian = IK.compute_jacobian(chains, skeleton)
    print("Jacobian:\n", jacobian)
    hessian = IK.compute_hessian(chains, skeleton, jacobian)
    print("\nHessian:", hessian)
    gradient = IK.compute_gradient(chains, skeleton, jacobian)
    print("Gradient:", gradient)

if __name__ == '__main__':
    show_simple_setup()
