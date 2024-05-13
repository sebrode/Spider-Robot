import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np
import polyscope as ps
import rainbow.simulators.inverse_kinematics.visualizer as GP

from scipy.optimize import minimize

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#


def show_simple_setup():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0.0), beta=0.0, gamma=0.0, tx=0.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=0.0, beta=0.0, gamma=0.0, tx=4.7, ty=0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=0.0, beta=0.0, gamma=0.0, tx=6.64, ty=0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=0.0, beta=0.0, gamma=0.0, tx=12.653, ty=0, tz=0.0)
    
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)

    def compute_jac(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_jacobian(chains, skeleton)

    def compute_hess(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_hessian(chains, skeleton, compute_jac(x, chains, skeleton))

    def compute_grad(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_gradient(
            chains, skeleton, compute_jac(x, chains, skeleton))

    def compute_obj(x, chains, skeleton):
        IK.set_joint_angles(skeleton, x)
        IK.update_skeleton(skeleton)
        return IK.compute_objective(chains, skeleton)

    #IK.set_angle(B3.idx, 90, skeleton)
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

    # def constraint_1(x):
    #     return [x[2], x[5], x[8]]

    footPositionMatrix = np.array([
        V3.make(0, 0, 0),
        V3.make(0, 0, 0),
        V3.make(0, 0, 0)
    ])

    def simulateSpider(positionMatrix):
        totalAngleMatrix = np.zeros(
            (len(IK.get_joint_angles(skeleton)), len(positionMatrix)))
        totalObjectMatrix = np.zeros(
            (len(positionMatrix), 1))

        for i in range(len(positionMatrix)):
            # IK.set_joint_angles(skeleton, positionMatrix[i, :].T)
            chains[0].goal = positionMatrix[i].T
            IK.update_skeleton(skeleton)
            result = minimize(
                fun=compute_obj,
                x0=IK.get_joint_angles(skeleton),
                # method='SLSQP',
                method='Newton-CG',
                jac=compute_grad,
                hess=compute_hess,
                args=(chains, skeleton),
                # constraints={'type': 'eq', 'fun': constraint_1}
            )
            totalAngleMatrix[:, i] = result.x
            totalObjectMatrix[i, :] = result.fun

            # print("Optimal solution:", result.x)
            # print("Optimal value:", result.fun)
        return totalAngleMatrix, totalObjectMatrix

    angleMatrix, objectMatrix = simulateSpider(footPositionMatrix)
    # print(angleMatrix)
    # print(objectMatrix)
    print(np.array2string(angleMatrix, separator=', '))
    print(np.array2string(objectMatrix, separator=', '))

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


    S = GP.GraphicsComponent()
    S.generateSkeletonMesh(skeleton, chains)
    

        
    IK.solveVariables(chains, skeleton, 1000, 0.38, 0.0001, 0.05, 0.21, 0.0001)
    
    S.update_mesh(skeleton, chains)
    S.visualize()

if __name__ == '__main__':
    ps.init()
    show_simple_setup()
