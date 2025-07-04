from __future__ import annotations

from ctypes import c_int
from typing import Dict, TYPE_CHECKING, List, Tuple

import daqp
import numpy as np

from .connections import ActiveConnection, PassiveConnection
from .degree_of_freedom import DegreeOfFreedom
from .spatial_types import spatial_types as cas
from .spatial_types.derivatives import Derivatives

if TYPE_CHECKING:
    from .world import World
    from .world_entity import Body


class IKSolverException(Exception):
    pass


class UnreachableException(IKSolverException):
    def __init__(self, iterations: int):
        super().__init__(f'Converged after {iterations}, but target pose not reached.')


class MaxIterationsException(IKSolverException):
    def __init__(self, iterations: int):
        super().__init__(f'Failed to converge in {iterations} iterations.')


class QPSolverException(IKSolverException):
    flag_map = {
        2: 'Soft optimal',
        1: 'Optimal',
        -1: 'Infeasible',
        -2: 'Cycling detected',
        -3: 'Unbounded problem',
        -4: 'Iteration limit reached',
        -5: 'Nonconvex problem',
        -6: 'Initial working set overdetermined'
    }

    def __init__(self, exit_flag):
        self.exit_flag = exit_flag
        super().__init__(f'QP solver failed with exit flag: {self.flag_map[exit_flag]}')


class InverseKinematicsSolver:
    """
    Quadratic Programming-based Inverse Kinematics solver.

    This class handles the setup and solving of inverse kinematics problems
    using quadratic programming optimization.
    """
    _large_value = np.inf
    _convergence_tolerance = 1e-4

    def __init__(self, world: World):
        self.world = world
        self.iteration = -1

    def solve(self, root: Body, tip: Body, target: np.ndarray,
              dt: float = 0.05, max_iterations: int = 200,
              translation_velocity: float = 0.2, rotation_velocity: float = 0.2) -> Dict[DegreeOfFreedom, float]:
        """
        Solve inverse kinematics problem.

        :param root: Root body of the kinematic chain
        :param tip: Tip body of the kinematic chain
        :param target: Desired tip pose relative to the root body
        :param dt: Time step for integration
        :param max_iterations: Maximum number of iterations
        :param translation_velocity: Maximum translation velocity
        :param rotation_velocity: Maximum rotation velocity
        :return: Dictionary mapping DOF names to their computed positions
        """
        qp_problem = QPProblem(
            world=self.world,
            root=root,
            tip=tip,
            target=target,
            dt=dt,
            translation_velocity=translation_velocity,
            rotation_velocity=rotation_velocity
        )

        # Initialize solver state
        solver_state = SolverState(
            position=np.array([self.world.state[dof.name].position for dof in qp_problem.active_dofs]),
            passive_position=np.array([self.world.state[dof.name].position for dof in qp_problem.passive_dofs])
        )

        # Run iterative solver
        final_position = self._solve_iteratively(qp_problem, solver_state, dt, max_iterations)

        return {dof: final_position[i] for i, dof in enumerate(qp_problem.active_dofs)}

    def _solve_iteratively(self, qp_problem: QPProblem, solver_state: SolverState, dt: float,
                           max_iterations: int) -> np.ndarray:
        """Run the main IK solver loop."""
        for self.iteration in range(max_iterations):
            velocity, slack = self._solve_qp_step(qp_problem, solver_state)

            if self._check_convergence(velocity, slack):
                break

            solver_state.update_position(velocity, dt)
        else:
            raise MaxIterationsException(max_iterations)
        return solver_state.position

    def _solve_qp_step(self, qp_problem: QPProblem, solver_state: SolverState) -> Tuple[np.ndarray, np.ndarray]:

        # Evaluate QP matrices at current state
        qp_matrices = qp_problem.evaluate_at_state(solver_state)

        # Setup constraint sense (equality for last 6 constraints)
        sense = np.zeros(qp_matrices.l.shape, dtype=c_int)
        sense[-6:] = 5  # equality constraints

        # Solve QP
        (xstar, fval, exitflag, info) = daqp.solve(
            qp_matrices.H, qp_matrices.g, qp_matrices.A,
            qp_matrices.u, qp_matrices.l, sense
        )

        if exitflag != 1:
            raise QPSolverException(exitflag)

        return xstar[:len(qp_problem.active_symbols)], xstar[len(qp_problem.active_symbols):]

    def _check_convergence(self, velocity: np.ndarray, slack: np.ndarray) -> bool:
        vel_below_threshold = np.max(np.abs(velocity)) < self._convergence_tolerance
        slack_below_threshold = np.max(np.abs(slack)) < self._convergence_tolerance
        if vel_below_threshold and slack_below_threshold:
            return True
        if vel_below_threshold and not slack_below_threshold:
            raise UnreachableException(self.iteration)
        return False


class QPProblem:
    """
    Represents a quadratic programming problem for inverse kinematics.
    """

    def __init__(self, world: World, root: Body, tip: Body, target: np.ndarray, dt: float, translation_velocity: float,
                 rotation_velocity: float):
        self.world = world
        self.root = root
        self.tip = tip
        self.target = target
        self.dt = dt
        self.translation_velocity = translation_velocity
        self.rotation_velocity = rotation_velocity

        # Extract DOFs and setup problem
        self.active_dofs, self.passive_dofs, self.active_symbols, self.passive_symbols = self._extract_dofs()
        self._setup_constraints()
        self._setup_weights()
        self._compile_functions()

    def _extract_dofs(self) -> Tuple[list[DegreeOfFreedom], list[DegreeOfFreedom], list[cas.Symbol], list[cas.Symbol]]:
        """Extract active and passive DOFs from the kinematic chain."""
        active_dofs = []
        passive_dofs = []

        for connection in self.world.compute_chain_of_connections(self.root, self.tip):
            if isinstance(connection, ActiveConnection):
                active_dofs.extend(connection.active_dofs)
            if isinstance(connection, PassiveConnection):
                passive_dofs.extend(connection.passive_dofs)

        active_dofs = list(sorted(active_dofs, key=lambda d: str(d.name)))
        passive_dofs = list(sorted(passive_dofs, key=lambda d: str(d.name)))

        active_symbols = [dof.position_symbol for dof in active_dofs]
        passive_symbols = [dof.position_symbol for dof in passive_dofs]

        return active_dofs, passive_dofs, active_symbols, passive_symbols

    def _setup_constraints(self):
        """Setup all constraints for the QP problem."""
        self.constraint_builder = ConstraintBuilder(
            self.world, self.root, self.tip, self.target,
            self.dt, self.translation_velocity, self.rotation_velocity
        )

        # Box constraints
        self.lower_box_constraints, self.upper_box_constraints = self.constraint_builder.build_box_constraints(
            self.active_dofs
        )
        self.box_constraint_matrix = cas.eye(len(self.lower_box_constraints))

        # Goal constraints
        self.eq_bound_expr, self.neq_matrix = self.constraint_builder.build_goal_constraints(
            self.active_symbols
        )

        # Combine constraints
        self.l = cas.vstack([self.lower_box_constraints, self.eq_bound_expr])
        self.u = cas.vstack([self.upper_box_constraints, self.eq_bound_expr])
        self.A = cas.vstack([self.box_constraint_matrix, self.neq_matrix])

    def _setup_weights(self):
        """Setup quadratic and linear weights for the QP problem."""
        dof_weights = [0.001 * (1. / min(1, dof.get_upper_limit(Derivatives.velocity))) ** 2
                       for dof in self.active_dofs]
        slack_weights = [2500 * (1. / 0.2) ** 2] * 6

        self.quadratic_weights = cas.Expression(dof_weights + slack_weights)
        self.linear_weights = cas.zeros(*self.quadratic_weights.shape)

    def _compile_functions(self):
        """Compile all symbolic expressions into functions."""
        symbol_args = [self.active_symbols, self.passive_symbols]

        self.l_f = self.l.compile(symbol_args)
        self.u_f = self.u.compile(symbol_args)
        self.A_f = self.A.compile(symbol_args)
        self.quadratic_weights_f = self.quadratic_weights.compile(symbol_args)
        self.linear_weights_f = self.linear_weights.compile(symbol_args)

    def evaluate_at_state(self, solver_state) -> QPMatrices:
        """Evaluate QP matrices at the current solver state."""
        return QPMatrices(
            l=self.l_f.fast_call(solver_state.position, solver_state.passive_position),
            u=self.u_f.fast_call(solver_state.position, solver_state.passive_position),
            A=self.A_f.fast_call(solver_state.position, solver_state.passive_position),
            H=np.diag(self.quadratic_weights_f.fast_call(solver_state.position, solver_state.passive_position)),
            g=self.linear_weights_f.fast_call(solver_state.position, solver_state.passive_position)
        )


class ConstraintBuilder:
    """
    Builds constraints for the inverse kinematics QP problem.
    """

    def __init__(self, world: World, root: Body, tip: Body, target: np.ndarray, dt: float, translation_velocity: float,
                 rotation_velocity: float):
        self.world = world
        self.root = root
        self.tip = tip
        self.target = target
        self.dt = dt
        self.translation_velocity = translation_velocity
        self.rotation_velocity = rotation_velocity
        self.large_value = np.inf

    def build_box_constraints(self, active_dofs: List[DegreeOfFreedom]) -> Tuple[cas.Expression, cas.Expression]:
        """Build position and velocity limit constraints for DOFs."""
        lower_constraints = []
        upper_constraints = []

        for dof in active_dofs:
            ll = cas.max(-1, dof.get_lower_limit(Derivatives.velocity))
            ul = cas.min(1, dof.get_upper_limit(Derivatives.velocity))

            if dof.has_position_limits():
                ll = cas.max(dof.get_lower_limit(Derivatives.position) - dof.position_symbol, ll)
                ul = cas.min(dof.get_upper_limit(Derivatives.position) - dof.position_symbol, ul)

            lower_constraints.append(ll)
            upper_constraints.append(ul)

        # Add slack variables
        lower_constraints.extend([-self.large_value] * 6)
        upper_constraints.extend([self.large_value] * 6)

        return cas.Expression(lower_constraints), cas.Expression(upper_constraints)

    def build_goal_constraints(self, active_symbols: List[cas.Symbol]) -> Tuple[cas.Expression, cas.Expression]:
        """Build position and rotation goal constraints."""
        root_T_tip = self.world.compose_forward_kinematics_expression(self.root, self.tip)

        # Position and rotation errors
        position_error = self._compute_position_error(root_T_tip)
        rotation_error = self._compute_rotation_error(root_T_tip)

        # Current state and jacobian
        current_expr = cas.vstack([
            root_T_tip.to_position()[:3],
            root_T_tip.to_rotation().to_quaternion()[:3]
        ])
        eq_bound_expr = cas.vstack([position_error, rotation_error])

        J = cas.jacobian(current_expr, active_symbols)
        neq_matrix = cas.hstack([J * self.dt, cas.eye(6) * self.dt])

        return eq_bound_expr, neq_matrix

    def _compute_position_error(self, root_T_tip: cas.TransformationMatrix) -> cas.Expression:
        """Compute position error with velocity limits."""
        root_P_tip = root_T_tip.to_position()
        root_T_tip_goal = cas.TransformationMatrix(self.target)
        root_P_tip_goal = root_T_tip_goal.to_position()

        translation_cap = self.translation_velocity * self.dt
        position_error = root_P_tip_goal[:3] - root_P_tip[:3]

        for i in range(3):
            position_error[i] = cas.limit(position_error[i], -translation_cap, translation_cap)

        return position_error

    def _compute_rotation_error(self, root_T_tip: cas.TransformationMatrix) -> cas.Expression:
        """Compute rotation error with velocity limits."""
        rotation_cap = self.rotation_velocity * self.dt
        hack = cas.RotationMatrix.from_axis_angle(cas.Vector3((0, 0, 1)), -0.0001)
        root_R_tip = root_T_tip.to_rotation().dot(hack)

        root_T_tip_goal = cas.TransformationMatrix(self.target)
        root_Q_tip_goal = root_T_tip_goal.to_quaternion()
        root_Q_tip = root_R_tip.to_quaternion()

        # Ensure quaternion consistency
        root_Q_tip_goal = cas.if_less(root_Q_tip_goal.dot(root_Q_tip), 0, -root_Q_tip_goal, root_Q_tip_goal)

        rotation_error = root_Q_tip_goal[:3] - root_Q_tip[:3]
        for i in range(3):
            rotation_error[i] = cas.limit(rotation_error[i], -rotation_cap, rotation_cap)

        return rotation_error


class SolverState:
    """
    Represents the state of the IK solver during iteration.
    """

    def __init__(self, position: np.ndarray, passive_position: np.ndarray):
        self.position = position
        self.passive_position = passive_position
        self.positions_history = []
        self.velocities_history = []

    def update_position(self, velocity: np.ndarray, dt: float):
        self.positions_history.append(self.position.copy())
        self.velocities_history.append(velocity.copy())
        self.position += velocity * dt


class QPMatrices:
    """
    Container for QP problem matrices at a specific state.
    """

    def __init__(self, l, u, A, H, g):
        self.l = l
        self.u = u
        self.A = A
        self.H = H
        self.g = g
