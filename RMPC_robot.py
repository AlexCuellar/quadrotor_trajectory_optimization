from time import time
from gurobipy import Model, Var, MVar, GRB
from numpy import dot, empty, sqrt, array
from scipy.special import erfinv

class LCC:
    """
    Represents a Linear Chance Constraint (LCC), or namely a chance constraint
    defined over a linear inequality.
    """

    def __init__(self, k, h, g, delta, id, use_milp=False, M_i=None, M_j=None):
        self.k = k # timestep
        self.h = h # vector
        self.g = g # limit
        self.delta = delta # probability of failure
        self.use_milp = use_milp 
        self.M_i = M_i # M-variable timestep 
        self.M_j = M_j # M-variable side
        self.id = id # Object ID number


class RMPC:
    """
    A class that can represent a robust model-predicitve control problem.
    It can also convert it to a determinized version.
    """

    def __init__(self, timesteps, xdim, xlims, udim, ulims, A, B, x0_bar, Sigma_x0, Sigma_w, Delta, xf_bar):
        """
        Initializes the class. Takes in the following inputs:
        
            timesteps - an integer number of timesteps (such as 5)

            xdim - the dimensionality of the state vector x. It will be stored as an xdim column vector.

            xlims - a list of limits on x, the problems state variables. Each item in the list
                    should be a tuple of ranges, like (20, 30), which means that 20 <= x_i <= 30
                    across all timesteps. If xlims contains just a single tuple, it is applied to all
                    xdim of the state variables in x. If xlims contains xdim tuples, then each is applied
                    individually. Anything else results in an error.

            udim - the dimensionality of the control vector u.

            ulims - same idea as xlims, but for u instead of x.

            A, B - the matrices that specify the discrete-time dynamics of the system, which take the form
                   x_{k+1} = A x_{k} + B u_{k}. Specified as NumPy matrices. `A` should be xdim x xdim in 
                   size, and `B` should be xdim x udim.

            x0_bar, Sigma_x0 - specify the initial distribuition of x at timestep k=0.
                               x is assumed to be normally distributed about N(x0_bar, Sigma_x0).
                               x0_bar should be a vector of length xdim, and Sigma_x0 should be
                               an xdim x xdim matrix.

            Sigma_w - the covariance matrix of the additive noise w_k (which is assumed to have zero mean).
                      This should be an xdim x xdim matrix.

            Delta - the overall chance constraint.
        """
        # We may either have one set of limits for all indices or a set of limits for each index
        self.xdim = xdim
        self.udim = udim
        self.timesteps = timesteps
        if len(xlims) != 1 and len(xlims) != xdim:
            raise Exception('Incorrectly specified x limits.')
        if len(ulims) != 1 and len(ulims) != udim:
            raise Exception('Incorrectly specified u limits.')

        # Create a linear program to be solved
        self._problem = Model("Optimizer")

        # A list of linear chance constraints (lcc's)
        self.lcc = []

        # Arrays/matrices to hold variables and covariances
        self._xbar =  self._problem.addMVar((self.timesteps + 1, self.xdim), vtype=GRB.CONTINUOUS)
        self._u =     self._problem.addMVar((self.timesteps    , self.udim), vtype=GRB.CONTINUOUS)
        self.abs_u =  self._problem.addMVar((self.timesteps    , self.udim), vtype=GRB.CONTINUOUS)
        self._Sigma_x = empty((timesteps + 1), dtype=object)
        self._Obj_Vars = [] # self._problem.addMVar((self.timesteps + 1, 6), vtype=GRB.BINARY)

        for k in range(self.timesteps + 1):
            for i in range(self.xdim):
                self._xbar[k, i].lb = xlims[i][0]
                self._xbar[k, i].ub = xlims[i][1]

        for k in range(self.timesteps):
            for i in range(self.udim):
                self._u[k, i].lb = ulims[i][0]
                self._u[k, i].ub = ulims[i][1]

        # Store RMPC problem parameters
        self.A = A
        self.B = B
        self.x0_bar = x0_bar
        self.Sigma_x0 = Sigma_x0
        self.Sigma_w = Sigma_w
        self.Delta = Delta
        self.xf_bar = xf_bar
        self.M_val = 1000

        # Store the solved / unsolved (or no solution) status.
        self.status = -1 # Not solved

    def add_lcc(self, k, h, g, delta, id):
        """Create and store a linear chance constraint."""
        self.lcc.append(LCC(k, h, g, delta, id))

    def add_object(self, obj_center, obj_size, delta, id):
        '''Add Linear Chance Constraints (LCCs) necessary to define an obstacle to avoid'''
        self._Obj_Vars.append(self._problem.addMVar((self.timesteps + 1, 6), vtype=GRB.BINARY))
        h_x = array([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
        h_y = array([0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
        h_z = array([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        inflate = [.2, .1, .1]
        x_lb = obj_center[0] - obj_size[0]/2 - inflate[0]
        x_ub = obj_center[0] + obj_size[0]/2 + inflate[0]
        y_lb = obj_center[1] - obj_size[1]/2 - inflate[1]
        y_ub = obj_center[1] + obj_size[1]/2 + inflate[1]
        z_lb = obj_center[2] - obj_size[2]/2 - inflate[2]
        z_ub = obj_center[2] + obj_size[2]/2 + inflate[2]
        for k in range(self.timesteps + 1):
            self.lcc.append(LCC(k,  h_x,  x_lb, delta, id, True, k, 0))
            self.lcc.append(LCC(k, -h_x, -x_ub, delta, id, True, k, 1))
            self.lcc.append(LCC(k,  h_y,  y_lb, delta, id, True, k, 2))
            self.lcc.append(LCC(k, -h_y, -y_ub, delta, id, True, k, 3))
            self.lcc.append(LCC(k,  h_z,  z_lb, delta, id, True, k, 4))
            self.lcc.append(LCC(k, -h_z, -z_ub, delta, id, True, k, 5))

    def _compute_variance(self):
        """
        Compute variances at all timesteps using problem parameters. Note that these
        can be computed in advance (as is done here), since the variance will continue to grow
        and doesn't depend at all on the control variable values, or any other values.
        """
        self._Sigma_x[0] = self.Sigma_x0
        for k in range(self.timesteps):
            self._Sigma_x[k+1] = dot(dot(self.A, self._Sigma_x[k]), self.A.transpose()) + self.Sigma_w

    def _encode_linear_dynamics(self):
        """Encode linear dynamic constraints."""
        (rA, cA) = self.A.shape
        (rB, cB) = self.B.shape
        (rx, cx) = self._xbar.shape
        (ru, cu) = self._u.shape

        if cA != cx:
            raise Exception('Shape of A inconsistent with x_(k).')
        if cB != cu:
            raise Exception('Shape of B inconsistent with u_(k).')
        if rA != rB:
            raise Exception('Shape of A inconsistent with shape of B.')

        # x_{k+1} = A*x_{k} + B*u_{k}
        for k in range(ru):
            # for i in range(cx):
            self._problem.addConstr(self._xbar[k+1, :] == self.A @ self._xbar[k, :] + self.B @ self._u[k, :])

        # Initial state
        self._problem.addConstr(self._xbar[0, :] == self.x0_bar)
        # Final state
        self._problem.addConstr(self._xbar[self.timesteps, :] == self.xf_bar)

        # A trick to compute the absolute value of control with linear inequalities
        for k in range(ru):
            for i in range(cu):
                self._problem.addConstr(self.abs_u[k, i] >= self._u[k, i])
                self._problem.addConstr(self.abs_u[k, i] >= -1*self._u[k, i])

    def _encode_probabilistic_constraints(self):
        """Encode chance constraints as deterministic constraints."""
        for con in self.lcc:
            if con.delta == None:
                raise Exception('Missing risk allocation on probabilistic linear constraint.')
            if not con.use_milp:
                self._problem.addConstr(con.h @ self._xbar[con.k, :] <= con.g - sqrt(2*dot(dot(con.h, self._Sigma_x[con.k]), con.h)) * erfinv(1 - 2*con.delta)) 
            else:
                self._problem.addConstr(con.h @ self._xbar[con.k, :] <= con.g + self._Obj_Vars[con.id][con.k, con.M_j]*self.M_val)
                # self._problem.addConstr(con.h @ self._xbar[con.k, :] <= con.g - sqrt(2*dot(dot(con.h, self._Sigma_x[con.k]), con.h)) * erfinv(1 - 2*con.delta) + self._M[con.k, con.M_j]*self.M_val)  

    def _encode_objective_function(self):
        """Encode the objective: minimizing the sum of the absolute values of all control variables."""
        self._problem.setObjective(self.abs_u.sum())

    def _encode_M_constraint(self):
        for obj_id in range(len(self._Obj_Vars)):
            for k in range(self.timesteps + 1):
                self._problem.addConstr(5 == self._Obj_Vars[obj_id][k, :].sum())

    def _solve(self):
        """Solve the problem, and raise an exception if it is unsolvable."""
        self.status = self._problem.optimize()
        if self.status == -1:
            raise Exception('No solution found.')

    def determinize_and_solve(self):
        """
        This method encodes all of the constraints, determinizes and then encodes all of the
        probabilistic constraints (i.e., the LCC's), and then solves the determinized system.
        """
        self._compute_variance()
        self._encode_linear_dynamics()
        self._encode_probabilistic_constraints()
        self._encode_objective_function()
        self._encode_M_constraint()
        self._solve()

    def objective(self):
        """
        Returns the objective value of the solved problem, 
        or an exception if it hasn't been solved yet.
        """
        if self.status == -1:
            raise Exception('Please call determinize_and_solve() successfully before calling objective()!')
        return self._problem.getObjective().getValue()

    def xbar(self, k):
        """
        Returns the solved value for xbar (the mean of x) at timestep k.
            Input:
                k - an integer from 0 to timesteps
        Throws an error if the system hasn't been solved successfully yet.
        """
        if self.status == -1:
            raise Exception('Please call determinize_and_solve() successfully before calling xbar()!')
        return array([xi.X for xi in self._xbar[k, :]])

    def Sigma_x(self, k):
        """
        Returns the value for Sigma_x (the covariance matrix of x) at timestep k.
            Input:
                k - an integer from 0 to timesteps
        Throws an error if the system hasn't been solved successfully yet.
        """
        if self.status == -1:
            raise Exception('Please call determinize_and_solve() successfully before calling Sigma_x()!')
        return self._Sigma_x[k]

    def u(self, k):
        """
        Returns the solved value for u (the control value) at timestep k.
            Input:
                k - an integer from 0 to timesteps - 1
        Throws an error if the system hasn't been solved successfully yet.
        """
        if self.status == -1:
            raise Exception('Please call determinize_and_solve() successfully before calling u()!')
        return array([ui.X for ui in self._u[k, :]])

    def M(self, k):
        """
        Returns the solved value for M (the control value) at timestep k.
            Input:
                k - an integer from 0 to timesteps - 1
        Throws an error if the system hasn't been solved successfully yet.
        """
        if self.status == -1:
            raise Exception('Please call determinize_and_solve() successfully before calling M()!') 
        # print(self._M[0, 0])
        object_Ms = []
        for obj in self._Obj_Vars:
            object_Ms.append([mi.X for mi in obj[k, :]])
        return object_Ms
