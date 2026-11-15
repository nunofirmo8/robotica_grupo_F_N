"""Labwork 2 - algebraic inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        inverse_kinematics(T_d)                    every distinct solution for a pose
        select_solution(solutions, q_current)      the solution that is executed

    Part B - the node
        IkNode                                     reads a desired pose, publishes
                                                the selected joint angles

Run the node with ``ros2 run lite6_ik ik_node``.

``transform_from_pose``, ``JOINT_LIMITS``, the constants ``TOL_SAME`` and
``TOL_WRIST`` and ``main`` below are PROVIDED: already implemented, do not edit
them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
    ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, in the controller's convention, as published on
    /joint_states (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. ``test/test_inverse_kinematics.py`` checks the imports.
"""

import math

import numpy as np

from lite6_kinematics.forward_kinematics import dh_table, forward_kinematics

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:         # the maths and its unit tests also run without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'


# =============================================================================
# Part B - the maths
# =============================================================================
# Parâmetros DH lidos da nossa forward_kinematics (Trabalho 1), para a IK usar
# exatamente os mesmos números. Linha i-1 da tabela: [a_{i-1}, alpha_{i-1}, d_i, theta_i].
_DH0 = dh_table(np.zeros(6))
D1 = float(_DH0[0, 2])          # altura do ombro
A2 = float(_DH0[2, 0])          # braço (ombro -> cotovelo)
A3 = float(_DH0[3, 0])          # antebraço em L: perna a3 ...
D4 = float(_DH0[3, 2])          # ... e perna d4
D6 = float(_DH0[5, 2])          # centro do punho -> link_eef
L3 = math.hypot(A3, D4)         # antebraço como uma reta (cotovelo -> centro do punho)
PHI = math.atan2(D4, A3)        # ângulo fixo entre a perna a3 e essa reta
REACH_TOL = 1e-9                # tolerância do critério de alcance (arredondamento)


def _wrap(a: float) -> float:
    """Ângulo em (-pi, pi]."""
    a = math.atan2(math.sin(a), math.cos(a))
    return math.pi if a <= -math.pi else a


def _same_solution(qa, qb) -> bool:
    """True se nenhum ângulo difere mais de TOL_SAME (módulo 2 pi)."""
    return all(abs(_wrap(x - y)) <= TOL_SAME for x, y in zip(qa, qb))


def inverse_kinematics(T_d) -> list:
    """Every distinct joint vector that puts the end-effector at T_d.

    The solution is closed-form, from the expressions of Part A: no iterative
    or optimisation method.

    Args:
        T_d: (4, 4) desired pose of {6} in {0}; translation in metres.

    Returns:
        A list of (6,) float arrays, joint angles in radians, order
        joint1..joint6, one array per distinct solution (in any order). The
        empty list if T_d is out of the robot's reach.

    Conventions of Part A:
        * every angle is in (-pi, pi]; the joint ranges JOINT_LIMITS are NOT
        applied here: a solution outside them is still returned;
        * two solutions in which no angle differs by more than TOL_SAME
        (angles compared modulo 2 pi) are the same solution: only one of
        them is returned;
        * if the angle between the axes of joints 4 and 6 is smaller than
        TOL_WRIST, those axes are aligned and every solution returned has
        q4 = 0.
    """
    T_d = np.asarray(T_d, dtype=float)
    R, p = T_d[:3, :3], T_d[:3, 3]

    # Passo 1 - centro do punho: recua d6 ao longo do eixo z da ferramenta.
    pw = p - D6 * R[:, 2]
    s = pw[2] - D1                          # altura acima do ombro
    r_xy = math.hypot(pw[0], pw[1])         # distância horizontal ao eixo da base
    base = math.atan2(pw[1], pw[0])

    solutions = []
    # Passo 2 - q1: o plano do braço passa pelo punho (à frente ou atrás).
    for q1, rho in ((base, r_xy), (base + math.pi, -r_xy)):

        # Passo 3 - q3: lei dos cossenos, sin(q3 - phi) = c.
        c = (rho * rho + s * s - A2 * A2 - L3 * L3) / (2.0 * A2 * L3)
        if abs(c) > 1.0 + REACH_TOL:
            return []                       # fora do alcance
        c = max(-1.0, min(1.0, c))          # tolera o arredondamento na fronteira
        root = math.sqrt(1.0 - c * c)

        for elbow in (1.0, -1.0):           # cotovelo para um lado / para o outro
            x = math.atan2(c, elbow * root)     # x = q3 - phi
            q3 = x + PHI

            # Passo 4 - q2: (s, rho) é o vetor (A, B) rodado de q2.
            A = A2 + L3 * math.sin(x)
            B = L3 * math.cos(x)
            q2 = math.atan2(rho, s) - math.atan2(B, A)

            # Passo 5 - orientação: M = R_3^6 = (R_0^3)^T R.
            R03 = np.asarray(forward_kinematics([q1, q2, q3, 0.0, 0.0, 0.0],
                                                all_frames=True)[3])[:3, :3]
            M = R03.T @ R
            # M[1,2] = -cos q5,  M[0,2] = -sin q5 cos q4,  M[2,2] = -sin q5 sin q4,
            # M[1,0] = -sin q5 cos q6,  M[1,1] = sin q5 sin q6
            sin5 = math.hypot(M[0, 2], M[2, 2])         # |sin q5|
            wrist = math.atan2(sin5, -M[1, 2])          # ângulo entre os eixos 4 e 6

            if wrist < TOL_WRIST:
                # Singular (q5 = 0): só q4 + q6 é fixado; convenção q4 = 0.
                wrists = [(0.0, 0.0, math.atan2(M[2, 0], M[0, 0]))]
            elif wrist > math.pi - TOL_WRIST:
                # Eixos opostos (q5 = pi): só q6 - q4 é fixado; q4 = 0.
                wrists = [(0.0, math.pi, math.atan2(M[2, 0], -M[0, 0]))]
            else:
                wrists = []
                for sg in (1.0, -1.0):                  # punho normal / virado
                    q5 = math.atan2(sg * sin5, -M[1, 2])
                    q4 = math.atan2(-sg * M[2, 2], -sg * M[0, 2])
                    q6 = math.atan2(sg * M[1, 1], -sg * M[1, 0])
                    wrists.append((q4, q5, q6))

            for q4, q5, q6 in wrists:
                q = np.array([_wrap(v) for v in (q1, q2, q3, q4, q5, q6)], dtype=float)
                if not any(_same_solution(q, other) for other in solutions):
                    solutions.append(q)
    return solutions


def select_solution(solutions, q_current):
    """The solution that is executed, chosen among `solutions`.

    Args:
        solutions: list of (6,) joint vectors (rad), as inverse_kinematics()
                returns them. Their angles may be any real numbers: this
                function must not assume they are in (-pi, pi].
        q_current: six joint angles (rad), the current configuration of the arm.

    Returns:
        A (6,) float array, the selected joint vector, or None if no solution
        is executable (no solutions at all, or all discarded). The inputs are
        not modified.

    The rule is the one of Part A: in each solution, every angle q_i is
    replaced by the value q_i + 2 pi k (k an integer) that lies inside the
    range of joint i (JOINT_LIMITS) and is closest to q_current[i]; a solution
    for which no such value exists is discarded. Among the solutions that
    remain, the one at the smallest euclidean distance from q_current is
    selected, and the returned vector is that solution with the replaced angles.
    """
    q_current = np.asarray(q_current, dtype=float)
    two_pi = 2.0 * math.pi
    best, best_dist = None, math.inf
    for sol in solutions:
        candidate = np.empty(6)
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            qi, qa = float(sol[i]), float(q_current[i])
            k = round((qa - qi) / two_pi)           # múltiplo de 2 pi mais perto de qa
            values = [qi + two_pi * (k + d) for d in (-1, 0, 1)]
            values = [v for v in values if lo <= v <= hi]
            if not values:
                candidate = None                    # nenhum cabe na gama: descartada
                break
            candidate[i] = min(values, key=lambda v: abs(v - qa))
        if candidate is None:
            continue
        dist = float(np.linalg.norm(candidate - q_current))
        if dist < best_dist:
            best, best_dist = candidate, dist
    return best


# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node computes the solutions and
    selects one from the latest joint angles received. If there is a selected
    solution it publishes a JointState:
        name             JOINT_NAMES
        position         the selected solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                        ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs why:
        unreachable      the pose has no solution at all
        out_of_range     it has solutions, but all of them were discarded

    The topic names are constructor arguments so that the tests can run the
    node on private topics: use them, do not write the names again.
    """

    def __init__(self, target_topic='/lite6_ik/target', joint_states_topic='/joint_states',
                solution_topic='/lite6_ik/solution'):
        super().__init__('ik_node')
        self.q_current = None                       # últimos ângulos de junta recebidos
        self.pub = self.create_publisher(JointState, solution_topic, 10)
        self.create_subscription(JointState, joint_states_topic, self.on_joint_state, 10)
        self.create_subscription(PoseStamped, target_topic, self.on_target, 10)

    def on_joint_state(self, msg):
        """Called for every JointState: keep the latest joint angles."""
        angles = dict(zip(msg.name, msg.position))
        if all(name in angles for name in JOINT_NAMES):
            self.q_current = np.array([angles[name] for name in JOINT_NAMES], dtype=float)

    def on_target(self, msg):
        """Called for every desired pose: solve it and publish the solution."""
        if self.q_current is None:
            self.get_logger().warn('no joint states received yet: target ignored')
            return
        T_d = transform_from_pose(msg)
        solutions = inverse_kinematics(T_d)
        if not solutions:
            self.get_logger().warn('unreachable: the desired pose has no solution')
            return
        q = select_solution(solutions, self.q_current)
        if q is None:
            self.get_logger().warn('out_of_range: every solution is outside the joint ranges')
            return
        out = JointState()
        out.header.stamp = msg.header.stamp         # o ik_check associa pela marca temporal
        out.header.frame_id = BASE_FRAME
        out.name = list(JOINT_NAMES)
        out.position = [float(v) for v in q]
        self.pub.publish(out)


# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
TOL_SAME = 1e-6     # rad - two solutions closer than this in every angle are the same
TOL_WRIST = 1e-6    # rad - angle between the axes of joints 4 and 6 below which they are aligned

# Joint ranges (rad) of the xarm_description model of the Lite 6, one row per
# joint, [lower, upper].
JOINT_LIMITS = np.array([
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.61799, 2.61799],
    [-0.061087, math.pi * 0.99],
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.1642, 2.1642],
    [-math.pi * 0.99, math.pi * 0.99],
])


def transform_from_pose(msg) -> np.ndarray:
    """(4, 4) transform of a geometry_msgs/PoseStamped.

    The quaternion is normalised first, so a slightly off unit quaternion still
    gives a rotation matrix.
    """
    p, o = msg.pose.position, msg.pose.orientation
    n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
    x, y, z, w = o.x / n, o.y / n, o.z / n, o.w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def main(args=None):
    """Entry point of ``ros2 run lite6_ik ik_node``."""
    if rclpy is None:
        raise SystemExit('ROS 2 is not available here: run this inside the container.')
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = IkNode()
        node.get_logger().info('ik_node running: desired poses in, joint angles out')
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it in lite6_ik/inverse_kinematics.py first.')
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
