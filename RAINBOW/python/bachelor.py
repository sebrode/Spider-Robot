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
    B0 = IK.create_root(skeleton, alpha=0.0, beta=0.0,
                        gamma=0.0, tx=0.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=0.0,
                     beta=0.0, gamma=0.0, tx=4.7, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=0.0,
                     beta=0.0, gamma=0.0, tx=6.64, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=0.0,
                     beta=0.0, gamma=0.0, tx=12.6530, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)

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

    def simulateSpider(positionMatrix):
        totalAngleMatrix = np.zeros(
            (len(IK.get_joint_angles(skeleton)), len(positionMatrix)))
        totalObjectMatrix = np.zeros(
            (len(positionMatrix), 1))

        for i in range(len(positionMatrix)):
            chains[0].goal = positionMatrix[i].T
            IK.update_skeleton(skeleton)
            result = minimize(
                fun=compute_obj,
                x0=IK.get_joint_angles(skeleton),
                method='SLSQP',
                # method='Newton-CG',
                jac=compute_grad,
                # hess=compute_hess,
                args=(chains, skeleton),
                bounds=((0, 0), (-np.pi/2, np.pi/2), (0, 0), (-np.pi/2, np.pi/2), (0, 0),
                        (0, 0), (-np.pi, 0), (0, 0), (0, 0), (-np.pi, np.pi), (0, 0), (0, 0))
                # options={'maxiter': 1000, 'disp': True}
                # constraints={'type': 'eq', 'fun': constraint_1}
            )
            totalAngleMatrix[:, i] = result.x
            totalObjectMatrix[i, :] = result.fun

            # print("Optimal solution:", result.x)
            # print("Optimal value:", result.fun)
        return totalAngleMatrix, totalObjectMatrix

    greyPositionMatrix = np.array([
        V3.make(12.0, -7.0, 0.0),
        V3.make(10.0, -5.0, 0.0),
        V3.make(6.0, -5.0, 0.0),
        V3.make(6.0, -7.0, 0.0),
        V3.make(10.0, -9.0, 0.0),
        V3.make(15.0, -7.0, 0.0),
        V3.make(10.0, -7.0, 0.0)
    ])
    greyAngleMatrix, greyObjectMatrix = simulateSpider(greyPositionMatrix)

    blackPositionMatrix = np.array([
        V3.make(12.0, -7.0, 0.0),
        V3.make(12.0, -5.0, 0.0),
        V3.make(15.0, -5.0, 0.0),
        V3.make(15.0, -7.0, 0.0),
        V3.make(10.0, -9.0, 0.0),
        V3.make(6.0, -7.0, 0.0),
        V3.make(10.0, -7.0, 0.0)
    ])
    blackAngleMatrix, blackObjectMatrix = simulateSpider(blackPositionMatrix)

    redPositionMatrix = np.array([
        V3.make(12.0, -7.0, 0.0),
        V3.make(10.0, -5.0, 0.0),
        V3.make(6.0, -5.0, 0.0),
        V3.make(6.0, -7.0, 0.0),
        V3.make(10.0, -9.0, 0.0),
        V3.make(15.0, -7.0, 0.0),
        V3.make(10.0, -7.0, 0.0)
    ])
    redAngleMatrix, redObjectMatrix = simulateSpider(redPositionMatrix)

    bluePositionMatrix = np.array([
        V3.make(12.0, -7.0, 0.0),
        V3.make(12.0, -5.0, 0.0),
        V3.make(15.0, -5.0, 0.0),
        V3.make(15.0, -7.0, 0.0),
        V3.make(10.0, -9.0, 0.0),
        V3.make(6.0, -7.0, 0.0),
        V3.make(10.0, -7.0, 0.0)
    ])
    blueAngleMatrix, blueObjectMatrix = simulateSpider(bluePositionMatrix)
    # print(angleMatrix)
    # print(objectMatrix)
    # x = [0.0, angleMatrix[0, -1], 0.0, angleMatrix[1, -1], 0.0, 0.0, angleMatrix[2, -1], 0.0, 0.0, angleMatrix[3, -1], 0.0, 0.0]
    # print(angleMatrix[[1,3,6,9],:])
    # print(np.array2string(angleMatrix[:, -1], separator=', '))
    fullAngleMatrix = np.array(
        [greyAngleMatrix[[1, 3, 6, 9], :], blackAngleMatrix[[1, 3, 6, 9], :], redAngleMatrix[[1, 3, 6, 9], :], blueAngleMatrix[[1, 3, 6, 9], :]])
    fullObjectMatrix = np.array(
        [greyObjectMatrix, blackObjectMatrix, redObjectMatrix, blueObjectMatrix])
    print(np.array2string(fullAngleMatrix, separator=', '))
    print(np.array2string(fullObjectMatrix, separator=', '))

    # jacobian = IK.compute_jacobian(chains, skeleton)
    # print("Jacobian:\n", jacobian)
    # hessian = IK.compute_hessian(chains, skeleton, jacobian)
    # print("\nHessian:", hessian)
    # gradient = IK.compute_gradient(chains, skeleton, jacobian)
    # print("Gradient:", gradient)
    # objective = IK.compute_objective(chains, skeleton)
    # print("Objective: ", objective)
    # end_effector = IK.get_end_effector(chains[0], skeleton)
    # print(end_effector)
    # jacobian = IK.compute_jacobian(chains, skeleton)
    # print("Jacobian:\n", jacobian)
    # hessian = IK.compute_hessian(chains, skeleton, jacobian)
    # print("\nHessian:", hessian)
    # gradient = IK.compute_gradient(chains, skeleton, jacobian)
    # print("Gradient:", gradient)


if __name__ == '__main__':
    show_simple_setup()
