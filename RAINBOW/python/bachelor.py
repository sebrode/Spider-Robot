import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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


def simulateSpider(positionMatrix):
    totalAngleMatrix = np.zeros(
        (len(IK.get_joint_angles(skeleton)), len(positionMatrix)))
    totalObjectMatrix = np.zeros(
        (len(positionMatrix), 1))

    for i in range(len(positionMatrix)):
        chains[0].goal = positionMatrix[i].T
        IK.update_skeleton(skeleton)
        result = IK.solve(
            fun=compute_obj,
            x0=IK.get_joint_angles(skeleton),
            method='SLSQP',
            # method='Newton-CG',
            jac=compute_grad,
            # hess=compute_hess,
            args=(chains, skeleton),
            bounds=((0, 0), (-np.pi/2, np.pi/2), (0, 0), (-np.pi/2, np.pi/2), (0, 0),
                    (0, 0), (-np.pi, 0), (0, 0), (0, 0), (-np.pi, np.pi), (0, 0), (0, 0))
        )
        totalAngleMatrix[:, i] = result.x
        totalObjectMatrix[i, :] = result.fun

    return totalAngleMatrix, totalObjectMatrix


greyPositionMatrix = np.array([
    V3.make(12.0, -7.0, 0.0),
    V3.make(8.0, -7.0, 4.0),
    V3.make(8.0, -7.0, 4.0),
    V3.make(8.0, -7.0, 4.0),
    V3.make(8.0, 0.0, 4.0),
    V3.make(6.0, 0.0, 10.0),
    V3.make(6.0, -7.0, 10.0),
    V3.make(12.0, -7.0, 0.0),
    V3.make(12.0, -7.0, 0.0),
    V3.make(12.0, -7.0, 0.0),
    V3.make(12.0, -7.0, 0.0)
])

blackPositionMatrix = np.array([
    V3.make(12.0, -3.0, 0.0),
    V3.make(12.0, 1.0, 0.0),
    V3.make(6.0, 1.0, -8.0),
    V3.make(6.0, -3.0, -8.0),
    V3.make(6.0, -3.0, -8.0),
    V3.make(6.0, -3.0, -8.0),
    V3.make(6.0, -3.0, -8.0),
    V3.make(12.0, -3.0, 0.0),
    V3.make(12.0, -3.0, 0.0),
    V3.make(12.0, -3.0, 0.0),
    V3.make(12.0, -3.0, 0.0)

])

redPositionMatrix = np.array([
    V3.make(12.0, -7.0, 0.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(3.0, -7.0, -12.0),
    V3.make(1.0, -7.0, -20.0),
    V3.make(1.0, -3.0, -20.0),
    V3.make(10.0, -7.0, -2.0),
    V3.make(10.0, -7.0, 0-2.0)
])

bluePositionMatrix = np.array([
    V3.make(12.0, -5.0, 2.0),
    V3.make(12.0, -5.0, 2.0),
    V3.make(12.0, -5.0, 2.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0),
    V3.make(2.0, -5.0, 12.0)
])

greyAngleMatrix, greyObjectMatrix = simulateSpider(greyPositionMatrix)
blackAngleMatrix, blackObjectMatrix = simulateSpider(blackPositionMatrix)
redAngleMatrix, redObjectMatrix = simulateSpider(redPositionMatrix)
blueAngleMatrix, blueObjectMatrix = simulateSpider(bluePositionMatrix)

fullAngleMatrix = np.array([greyAngleMatrix[[1, 3, 6, 9], :], blackAngleMatrix[[
    1, 3, 6, 9], :], redAngleMatrix[[1, 3, 6, 9], :], blueAngleMatrix[[1, 3, 6, 9], :]])
fullObjectMatrix = np.array(
    [greyObjectMatrix, blackObjectMatrix, redObjectMatrix, blueObjectMatrix])


def getAngleMatrix():
    return fullAngleMatrix


def getObjectMatrix():
    return fullObjectMatrix


print(np.array2string(fullAngleMatrix, separator=', '))
# print(np.array2string(fullObjectMatrix, separator=', '))
