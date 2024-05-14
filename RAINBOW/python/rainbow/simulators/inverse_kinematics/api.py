import rainbow.math.vector3 as V3
import rainbow.math.quaternion as Q
from rainbow.simulators.inverse_kinematics.types import *
from rainbow.math.angle import *
import numpy as np


def __is_valid_euler_code(euler_code):
    """
    Function to test if the Euler angle convention encoded as a string value is supported by this simulator.

    :param euler_code:   A text encoding of an euler angle encoding.
    :return:             If the Euler code is usable then the return value is true otherwise it is false.
    """
    proper_euler_anlges = ['XZX', 'XYX', 'YXY', 'YZY', 'ZYZ', 'ZXZ']
    tait_bryan_angles = ['XZY', 'XYZ', 'YXZ', 'YZX', 'ZYX', 'ZXY']
    if euler_code in proper_euler_anlges:
        return True
    if euler_code in tait_bryan_angles:
        return True
    return False


def create_skeleton():
    """
    This is a factory function for creating an empty IK skeleton.

    :return:  The skeleton that was created.
    """
    S = Skeleton()
    return S


def create_root(skeleton, alpha, tx, ty, tz, euler_code='ZYZ'):
    """
    This function creates a root bone in an empty skeleton. It should be the first function to use
    when populating a skeleton with bones. A skeleton can only have one root bone.

    :param skeleton:      The skeleton where to create the root bone.
    :param alpha:         The alpha Euler angle of the joint that is represented by this bone.
    :param tx:            The x-component of the bone vector.
    :param ty:            The y-component of the bone vector.
    :param tz:            The z-component of the bone vector.
    :param euler_code:    The Euler angle convention used for the joint represented by this bone. This
                          encodes the meaning of the alpha, beta and gamma joint angle values.
    :return:              A reference to the root bone that was created.
    """
    if skeleton.has_root():
        raise RuntimeError(
            'create_root(): Internal error, root already existed')
    if not __is_valid_euler_code(euler_code):
        raise RuntimeError(
            'create_root(): Internal error, invalid Euler angle code given')
    root = Bone()
    root.t = V3.make(tx, ty, tz)
    root.alpha = alpha
    root.euler_code = euler_code
    skeleton.bones.append(root)
    return root


def add_bone(skeleton, parent_idx, alpha, tx, ty, tz, euler_code='ZYZ'):
    """
    This function creates a new bone in a skeleton. It needs the index of an existing parent bone to know where to
    create the new bone.

    :param skeleton:      The skeleton where to create the new bone into.
    :param parent_idx:    The index of the parent bone.
    :param alpha:         The alpha Euler angle of the joint that is represented by this bone.
    :param tx:            The x-component of the bone vector.
    :param ty:            The y-component of the bone vector.
    :param tz:            The z-component of the bone vector.
    :param euler_code:    The Euler angle convention used for the joint represented by this bone. This
                          encodes the meaning of the alpha, beta and gamma joint angle values.
    :return:              A reference to the new bone that was created in the skeleton.
    """
    if not skeleton.has_bone(parent_idx):
        raise RuntimeError('add_bone(): Internal error, parent did not exist')
    if not __is_valid_euler_code(euler_code):
        raise RuntimeError(
            'create_root(): Internal error, invalid Euler angle code given')

    end_effector = Bone()
    end_effector.idx = len(skeleton.bones)
    end_effector.parent = parent_idx
    end_effector.t = V3.make(tx, ty, tz)
    end_effector.alpha = alpha
    end_effector.euler_code = euler_code

    skeleton.bones[parent_idx].children.append(end_effector.idx)
    skeleton.bones.append(end_effector)
    return end_effector


def __update_bone(bone, skeleton) -> None:
    """
    This function extract the joint angles of the current bone and use these to compute the relative transformation
    from parent bone to this bone. It then extracts the world coordinate pose of the parent bone and use all the
    information to compute the bones own world coordinate pose.

    Once this is done the function update the child bones world coordinate pose by invoking itself recursively.

    Hence, this function recursively traverse the skeleton from root to all end-effectors and while doing so it
    updates the world coordinate system pose of each bone in the skeleton.

    :param bone:          The current bone to update.
    :param skeleton:      The IK skeleton holding all the bones.
    :return:              None.
    """
    q_alpha = bone.get_rotation_alpha()  # Rotation based on alpha angle only

    t_parent = V3.zero()
    q_parent = Q.identity()
    if bone.has_parent():
        t_parent = skeleton.bones[bone.parent].t_wcs
        q_parent = skeleton.bones[bone.parent].q_wcs

    # Update this bone's world coordinate translation and rotation
    bone.t_wcs = t_parent + Q.rotate(q_parent, bone.t)
    bone.q_wcs = Q.prod(q_parent, q_alpha)  # Only use the alpha rotation

    for idx in bone.children:
        __update_bone(skeleton.bones[idx], skeleton)


def update_skeleton(skeleton) -> None:
    """
    This function makes sure that all bones in a skeleton get updated to have the correct world
    coordinate system pose.

    One should call this method if one have changed the joint angle values stored in the skeleton and now needs
    to know where things are located in the world coordinate system.

    :param skeleton:  The skeleton that must be updated.
    :return:          None.
    """
    __update_bone(skeleton.bones[0], skeleton)


def __make_chain(end_effector, skeleton):
    """
    This function traverses a skeleton from the given end-effectors. It creates
    a serial chain going from root bone to the given end-effector. Each chain stores the bone indices of all
    bones on the path from the root to the end-effector.

    :param end_effector:   A reference to the bone that one wish to use as an end-effector.
    :param skeleton:       The skeleton which the end-effector bone belongs to.
    :return:               The resulting IK chain.
    """
    chain = Chain()
    chain.skeleton = skeleton
    bone = end_effector
    while not bone.is_root():
        chain.bones.append(bone.idx)
        bone = skeleton.bones[bone.parent]
    chain.bones.append(bone.idx)
    chain.bones.reverse()
    return chain


def make_chains(skeleton):
    """
    A skeleton is basically a tree structure. When manipulating a skeleton with inverse kinematics one does this
    my specifying a goal for a sol-called end-effector. An end-effector can be any bone in the skeleton. However,
    end-effectors are often taken to be the leaves of the skeleton tree structure.

    We create this default set of end-effectors by identifying all leaves in the skeleton.

    Because we manipulate the skeleton my specifying goals for our end-effectors we will for each end-effector
    store this goal as well as all the bones on the path from the end-effector to the root bone of the
    skeleton. This path of bones makes up a serial chain from root to end-effector. Hence, we create a list
    of chains where each chain cooresponds to one end-effector.

    This wording of "chain" and "end-effector" comes from Robotics. The computer science equivalent would be
    something like "path" and "leave".

    :param skeleton:  The skeleton to create serial chains from.
    :return:          A list of all the default serial chains of the skeleton.
    """
    chains = []
    for bone in skeleton.bones:
        if bone.is_end_effector():
            chain = __make_chain(bone, skeleton)
            chains.append(chain)
    return chains


def get_joint_angles(skeleton):
    """
    This function retrieves all the joint angles from an IK-skeleton and returns them as a
    numpy array. This is convenient for converting from skeleton to numpy. The reverse conversion
    can be done by using the function set_joint_angles.

    :param skeleton:  The IK skeleton from where to retrieve the joint angles.
    :return:          A numpy array holding the joint angle values.
    """
    angles = np.zeros((len(skeleton.bones),), dtype=np.float64)
    for bone in skeleton.bones:
        angles[bone.idx + 0] = bone.alpha
    return angles


def set_joint_angles(skeleton, angles) -> None:
    """
    This is a convenience function that allow one to use a whole numpy array of joint angle values to
    set the joint angles stored in a skeleton data structure. This is nice for converting from numpy arrays
    to skeleton data structure.

    :param skeleton:       The IK skeleton where to set the joint angles.
    :param angles:         The numpy array that stores the joint angle values to use.
    :return:               None.
    """
    for bone in skeleton.bones:
        bone.alpha = angles[bone.idx + 0]


def get_end_effector(chain, skeleton):
    """
    This function retrieve the end-effector position of a chain in the world coordinate system (wcs). The function
     assumes the skeleton pose has been updated prior to invocation by using the function update_skeleton.

    :param chain:       The IK chain which the end-effector belongs to.
    :param skeleton:    The IK skeleton, assumed to have been updated by invoking update_skeleton.
    :return:            The 3D position of the end-effector in the world coordinate system.
    """
    t_wcs = skeleton.bones[chain.bones[-1]].t_wcs
    q_wcs = skeleton.bones[chain.bones[-1]].q_wcs
    tool = chain.tool
    return t_wcs + Q.rotate(q_wcs, tool)


def compute_jacobian(chains, skeleton):
    """
    Compute the Jacobian of the forward kinematic function. That is the function that computes the
    end-effector positions in world-space given the cufrrent joint-angle values.

    :param chains:    The IK chains defining the end-effectors.
    :param skeleton:  The IK skeleton.
    :return:          A numpy matrix with the value of the Jacobian.
    """
    num_angles = len(skeleton.bones)  # Only one angle per bone now
    num_coords = len(chains) * 3
    J = np.zeros((num_coords, num_angles), dtype=np.float64)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        for idx, bone in enumerate(skeleton.bones):
            q_parent = Q.identity()
            if bone.has_parent():
                q_parent = skeleton.bones[bone.parent].q_wcs

            q_alpha = bone.get_rotation_alpha()

            u = Q.rotate(q_parent, bone.get_axis_alpha())

            delta_p = e - bone.t_wcs

            J_alpha = np.cross(u, delta_p)

            # Update only the column for alpha since beta and gamma are removed
            J[row_offset:row_offset + 3, idx] = J_alpha
        row_offset += 3
    return J

# Assuming `chains` and `skeleton` are defined elsewhere in your code


def compute_hessian(chains, skeleton, J):
    """
    Computes the Hessian matrix using only alpha Euler angle for the IK problem.

    :param chains: List of IK chains.
    :param skeleton: Skeleton structure with joint angle values.
    :param J: Jacobian matrix computed using alpha angle only.
    :return: Hessian matrix.
    """
    H = np.dot(J.T, J)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r = chain.goal - e
        for k in chain.bones:
            Bk = skeleton.bones[k]
            q_parent = Q.identity()
            if Bk.has_parent():
                q_parent = skeleton.bones[Bk.parent].q_wcs

            q_alpha = Bk.get_rotation_alpha()
            u = Q.rotate(q_parent, Bk.get_axis_alpha())

            k_offset = k  # Adjust index to account for only alpha

            J_a = J[row_offset:row_offset + 3, k_offset]

            ua = np.dot(np.cross(u, J_a), r)

            # Since Hessian is symmetrical, we only need to update the upper triangle
            for h in range(k + 1):  # Ensure we only calculate each pair once
                Bh = skeleton.bones[h]
                h_offset = h  # Similarly adjusted

                J_b = J[row_offset:row_offset + 3, h_offset]

                ub = np.dot(np.cross(u, J_b), r)

                H[h_offset, k_offset] -= ub
                if h != k:
                    H[k_offset, h_offset] = H[h_offset, k_offset]  # Symmetry

        row_offset += 3
    return H


def compute_gradient(chains, skeleton, J):
    """
    This function compute the gradient of the IK objective function wrt all the joint angles.

    This implementation is based on an analytical solution for computing the gradient and is hence much faster than
    a numerical approximation method.

    :param chains:       The IK chains of the IK problem.
    :param skeleton:     The IK skeleton (basically storing the joint angle values)
    :param J:            The Jacobian matrix of the forward kinematics functions. Should be computed
                         using the function compute_jacobian.
    :return:             A numpy array holding the gradient value.
    """
    r = np.zeros((len(chains) * 3,), dtype=np.float64)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r[row_offset:row_offset + 3] = chain.goal - e
        row_offset += 3
    g = - np.dot(np.transpose(J), r)
    return g


def compute_objective(chains, skeleton) -> float:
    """
    This function computes the objective value for the IK problem represented by the IK chains. Each
    chain describe a goal for an end-effector of a serial IK chain. This function essentially
    measure the errors between corresponding goals and end-effectors and sum up all the errors to
    a single float value. A value of zero means all goals are fulfilled, a larger value means larger errors. Hence,
    to solve the IK problem one wish to minimize the objective function value as much as possible.

    :param chains:    The IK chains of the skeleton.
    :param skeleton:  The IK skeleton.
    :return:          The value of the IK objective function.
    """
    f = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r = chain.goal - e
        f += np.dot(r, r)
    return f * 0.5


def set_angle(idx, value, skeleton) -> None:
    """
    This function is a convenience function that sets the value of the idx-joint angle. This
    is such that end-users do not have to know about the data structures used.

    :param idx:        The index value of the joint angle (aka variable) that one which to set the value of.
    :param value:      The new value of the idx-joint-angle.
    :param skeleton:   The IK skeleton from which to set the angle value from.
    :return:           None
    """
    skeleton.bones[idx].alpha = value


def get_angle(idx, skeleton) -> float:
    """
    This function is a convenience function that extract the value of the idx-joint angle. This
    is such that end-users do not have to know about the data structures used.

    :param idx:        The index value of the joint angle (aka variable) that one which to retrive the value of.
    :param skeleton:   The IK skeleton from which to retrive the angle value from.
    :return:           The value of the idx-joint-angle.
    """
    return skeleton.bones[idx].alpha


def __numerical_differentiation_second_derivative(chains, skeleton, i, j, h):
    """
    Numerically evaluate the second order derivative of the objective function wrt the i and j joint angles.
    This method is not intended to be invoked directly by end-users.

    :param chains:         The IK chains of the IK skeleton.
    :param skeleton:       The IK skeleton.
    :param i:              The index of the first component (ie angle) that we compute the derivative with respect to.
    :param j:              The index of the second component (ie angle) that we compute the derivative with respect to.
    :param h:              The step-size to use for the finite difference approximation. We use a
                           central difference approximation.
    :return:               The second derivative of the IK objective function wrt variables i and j.
    """
    if i == j:
        update_skeleton(skeleton)
        f_i = compute_objective(chains, skeleton)
        theta_i = get_angle(i, skeleton)
        set_angle(i, theta_i + h, skeleton)
        update_skeleton(skeleton)
        f_iph = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        update_skeleton(skeleton)
        f_imh = compute_objective(chains, skeleton)
        set_angle(i, theta_i, skeleton)
        return (f_iph - 2 * f_i + f_imh) / (h * h)
    else:
        theta_i = get_angle(i, skeleton)
        theta_j = get_angle(j, skeleton)
        set_angle(i, theta_i + h, skeleton)
        set_angle(j, theta_j + h, skeleton)
        update_skeleton(skeleton)
        f_iph_jph = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        set_angle(j, theta_j + h, skeleton)
        update_skeleton(skeleton)
        f_imh_jph = compute_objective(chains, skeleton)
        set_angle(i, theta_i + h, skeleton)
        set_angle(j, theta_j - h, skeleton)
        update_skeleton(skeleton)
        f_iph_jmh = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        set_angle(j, theta_j - h, skeleton)
        update_skeleton(skeleton)
        f_imh_jmh = compute_objective(chains, skeleton)
        set_angle(i, theta_i, skeleton)
        set_angle(j, theta_j, skeleton)
        return (f_iph_jph - f_iph_jmh - f_imh_jph + f_imh_jmh) / (4 * h * h)


def __numerical_differentiation_first_derivative(chains, skeleton, i, h):
    """
    Numerically evaluate the derivative of the objective function wrt a single joint angle. This method is not
    intended to be invoked directly by end-users.

    :param chains:         The IK chains of the IK skeleton.
    :param skeleton:       The IK skeleton.
    :param i:              The index of the component (ie angle) that we compute the derivative with respect to.
    :param h:              The step-size to use for the finite difference approximation. We use a
                           central difference approximation.
    :return:               The derivative of the objective function wrt the i'th variable.
    """
    theta_i = get_angle(i, skeleton)
    set_angle(i, theta_i + h, skeleton)
    update_skeleton(skeleton)
    f_iph = compute_objective(chains, skeleton)
    set_angle(i, theta_i - h, skeleton)
    update_skeleton(skeleton)
    f_imh = compute_objective(chains, skeleton)
    set_angle(i, theta_i, skeleton)
    return (f_iph - f_imh) / (2 * h)


def compute_finite_difference_gradient(chains, skeleton, h=0.1):
    """
    This function uses finite difference method to get the gradient of the IK objective function.
    This is a slow method, and it is subject to numerical approximation error. Hence, it is far
    better to use the function compute_gradient instead.

    :param chains:          All the IK chains.
    :param skeleton:        The IK skeleton.
    :param h:               The step-size to use for the finite difference approximation. We
                            use a 1st order central difference approximation. In general as h goes
                            to zero the approximation error will go to zero as well.

    :return:                The gradient.
    """
    N = len(skeleton.bones) * 3
    g = np.zeros((N,), dtype=np.float64)
    for i in range(N):
        g[i] = __numerical_differentiation_first_derivative(
            chains, skeleton, i, h)
    return g


def compute_finite_difference_hessian(chains, skeleton, h=0.1):
    """
    This function uses finite difference method to get the gradient of the IK objective function.
    This is a slow method, and it is subject to numerical approximation error. Hence, it is far
    better to use the function compute_gradient instead.

    :param chains:          All the IK chains.
    :param skeleton:        The IK skeleton.
    :param h:               The step-size to use for the finite difference approximation. We
                            use a central difference approximation. In general as h goes
                            to zero the approximation error will go to zero as well.

    :return:                The Hessian.
    """
    N = len(skeleton.bones) * 3
    H = np.zeros((N, N), dtype=np.float64)
    for i in range(N):
        for j in range(N):
            H[i, j] = __numerical_differentiation_second_derivative(
                chains, skeleton, i, j, h)
    return H
