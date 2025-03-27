import os
import carb
import numpy as np
from numpy.linalg import svd
from numpy.linalg import det
from numpy.linalg import norm

from typing import Optional
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView

class Crazyflie(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "crazyflie",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.array] = None,
    ) -> None:
        """[summary]"""

        self._usd_path = usd_path
        self._name = name

        # Dynamically construct the asset path
        home_dir = os.path.expanduser("~")
        asset_dir = os.path.join(home_dir, "isaacsim_4_2/exts/omni.isaac.examples/omni/isaac/examples/user_examples/asset")
        asset_path = os.path.join(asset_dir, "cf2x_largex7.usd")

        # Ensure the file exists before adding it to the stage
        if not os.path.exists(asset_path):
            carb.log_error(f"Asset file not found: {asset_path}")
        else:
            add_reference_to_stage(asset_path, prim_path)

        """
        if self._usd_path is None:
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
            self._usd_path = assets_root_path + "/Isaac/Robots/Crazyflie/cf2x.usd"
        add_reference_to_stage(self._usd_path, prim_path)
        """

        # asset_path = "C:/Users/Vicon-OEM/Downloads/cf2x_largex7.usd"
        # add_reference_to_stage(asset_path, prim_path)

        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)


class CrazyflieView(ArticulationView):
    def __init__(self, prim_paths_expr: str, name: Optional[str] = "CrazyflieView") -> None:
        """[summary]"""

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
        )

        self.rigid_body =RigidPrimView(
            prim_paths_expr=f"/World/UAVs/.*/Crazyflie/body", 
            name=f"body_view")


        self.physics_rotors = [
            RigidPrimView(
                prim_paths_expr=f"/World/UAVs/.*/Crazyflie/m{i}_prop", 
                name=f"m{i}_prop_view")
            for i in range(1, 5)
        ]


def hat(x):
    ensure_vector(x, 3)

    hat_x = np.array([[0.0, -x[2], x[1]], \
                        [x[2], 0.0, -x[0]], \
                        [-x[1], x[0], 0.0]])
                    
    return hat_x


def vee(M):
    """Returns the vee map of a given 3x3 matrix.
    Args:
        x: (3x3 numpy array) hat of the input vector
    Returns:
        (3x1 numpy array) vee map of the input matrix
    """
    ensure_skew(M, 3)

    vee_M = np.array([M[2,1], M[0,2], M[1,0]])

    return vee_M


def deriv_unit_vector(A, A_dot, A_2dot):
    """Returns the unit vector and it's derivatives for a given vector.
    Args:
        A: (3x1 numpy array) vector
        A_dot: (3x1 numpy array) first derivative of the vector
        A_2dot: (3x1 numpy array) second derivative of the vector
    Returns:
        q: (3x1 numpy array) unit vector of A
        q_dot: (3x1 numpy array) first derivative of q
        q_2dot: (3x1 numpy array) second derivative of q
    """

    ensure_vector(A, 3)
    ensure_vector(A_dot, 3)
    ensure_vector(A_2dot, 3)

    nA = np.linalg.norm(A)

    if abs(np.linalg.norm(nA)) < 1.0e-9:
        raise ZeroDivisionError('The 2-norm of A should not be zero')

    nA3 = nA * nA * nA
    nA5 = nA3 * nA * nA

    A_A_dot = A.dot(A_dot)

    q = A / nA
    q_dot = A_dot / nA \
        - A.dot(A_A_dot) / nA3

    q_2dot = A_2dot / nA \
        - A_dot.dot(2.0 * A_A_dot) / nA3 \
        - A.dot(A_dot.dot(A_dot) + A.dot(A_2dot)) / nA3 \
        + 3.0 * A.dot(A_A_dot).dot(A_A_dot)  / nA5

    return (q, q_dot, q_2dot)


def ensure_vector(x, n):
    """Make sure the given input array x is a vector of size n.
    Args:
        x: (nx1 numpy array) vector
        n: (int) desired length of the vector
    Returns:
        True if the input array is satisfied with size constraint. Raises an
        exception otherwise.
    """

    np.atleast_2d(x)  # Make sure the array is atleast 2D.

    if not len(np.ravel(x)) == n:
        raise ValueError('Input array needs to be of length {}, but the size' \
            'detected is {}'.format(n, np.shape(x)))
    
    return True


def ensure_matrix(x, m, n):
    """Make sure the given input array x is a matrix of size mxn.
    Args:
        x: (mxn numpy array) array
        m: (int) desired number of rows
        n: (int) desired number of columns
    Returns:
        True if the input array is satisfied with size constraint. Raises an
        exception otherwise.
    """

    np.atleast_2d(x)  # Make sure the array is atleast 2D.

    if not np.shape(x) == (m, n):
        raise ValueError('Input array needs to be of size {} x {}, but the ' \
            'size detected is {}'.format(m, n, np.shape(x)))
    
    return True


def ensure_skew(x, n):
    """Make sure the given input array is a skew-symmetric matrix of size nxn.
    Args:
        x: (nxn numpy array) array
        m: (int) desired number of rows and columns
    Returns:
        True if the input array is a skew-symmetric matrix. Raises an
        exception otherwise.
    """
    ensure_matrix(x, n, n)
    
    if not np.allclose(x.T, -x):
        raise ValueError('Input array must be a skew-symmetric matrix')
    
    return True

class IntegralErrorVec3:
    def __init__(self):
        self.error = np.zeros(3)
        self.integrand = np.zeros(3)

    def integrate(self, current_integrand, dt):
        self.error += (self.integrand + current_integrand) * dt / 2.0
        self.integrand = current_integrand

    def set_zero(self):
        self.error = np.zeros(3)
        self.integrand = np.zeros(3)


class IntegralError:
    def __init__(self):
        self.error = 0.0
        self.integrand = 0.0

    def integrate(self, current_integrand, dt):
        self.error += (self.integrand + current_integrand) * dt / 2.0
        self.integrand = current_integrand

    def set_zero(self):
        self.error = 0.0
        self.integrand = 0.0


def ensure_SO3(R, tolerance=1e-6):
    """ Make sure the given input array is in SO(3).

    Args:
        x: (3x3 numpy array) matrix
        tolerance: Tolerance level for considering the magnitude as 1

    Returns:
        True if the input array is in SO(3). Raises an exception otherwise.
    """
    # Calculate the magnitude (norm) of the matrix
    magnitude = np.linalg.det(R)

    # R matrix should satisfy R^T@R = I and det(R) = 1:
    if np.allclose(R.T@R,np.eye(3),rtol=tolerance) and np.isclose(magnitude,1.,rtol=tolerance):
        return R
    else: 
        U, s, VT = psvd(R)
        R = U @ VT.T # Re-orthonormalized R
        return R
    
def psvd(A):
    assert A.shape == (3,3)
    U, s, VT = svd(A)
    detU = det(U)
    detV = det(VT)
    U[:,2] = U[:,2]*detU
    VT[2,:] = VT[2,:]*detV
    s[2] = s[2]*detU*detV
    # assert norm(A-U@np.diag(s)@VT) < 1e-7
    return U, s, VT.T

def q_to_R(q):
    """Converts a quaternion of a rotation matrix in SO(3).

    Args:
        q: (4x1 numpy array) quaternion in [x, y, z, w] format

    Returns:
        R: (3x3 numpy array) rotation matrix corresponding to the quaternion
    """

    # TODO: ensure quaternion instead of ensure vector
    ensure_vector(q, 4)

    R = np.identity(3)
    q13 = np.array([q[0], q[1], q[2]])
    q4 = q[3]

    hat_q = hat(q13)
    R += 2.0 * q4 * hat_q + 2.0 * hat_q.dot(hat_q)

    return R

def quaternion_to_rotation_matrix(quaternion):
    w, x, y, z = quaternion
    tx = 2.0 * x
    ty = 2.0 * y
    tz = 2.0 * z
    twx = tx * w
    twy = ty * w
    twz = tz * w
    txx = tx * x
    txy = ty * x
    txz = tz * x
    tyy = ty * y
    tyz = tz * y
    tzz = tz * z

    matrix = np.stack(
        [
            1 - (tyy + tzz),
            txy - twz,
            txz + twy,
            txy + twz,
            1 - (txx + tzz),
            tyz - twx,
            txz - twy,
            tyz + twx,
            1 - (txx + tyy),
        ],
        axis=-1,
    )
    matrix = np.reshape(matrix, (3, 3))
    return matrix

def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w ** 2 - 1.0).reshape(-1, 1)
    b = np.cross(q_vec, v, axis=-1) * q_w.reshape(-1, 1) * 2.0
    c = q_vec * np.matmul(q_vec.reshape(shape[0], 1, 3), v.reshape(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c