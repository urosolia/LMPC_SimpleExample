function [ x, u, v ] = FTOCP( x_t , N, Q, R, Qfun, SS, A, B, X, U, solver, invariantSet, bar_x_stored, bar_u_stored)
% FTOCP solves the Finite Time Optimal Control Problem
% The function takes as inputs
% - x_t: state of the system at time t
% - N: horizon length
% - (Q,R): matrices defining the running cost
% - Qfun: vector collecting the cost-to-go up to the current iteration
% - SS: matrix collecting the stored states up to the current iteration
% - (A,B): matrices defining the system dynamics
% - X: polyhedron representng the state constraints
% - U: polyhedron representng the input constraints
% - Solver: solver to use for the FTOCP
% - invariantSet: invariant set for the error dynamics
% - bar_x_stored: bar_x at the previous time step
% - bar_v_stored: nominal input at the previous time step

% Define Yalmip Variables
x=sdpvar(size(A,2)*ones(1,N+1),ones(1,N+1));
u=sdpvar(size(B,2)*ones(1,N),ones(1,N));
if isempty(bar_x_stored)
    v = 0;
else
    v=sdpvar(1);
end
lambda = sdpvar(length(Qfun), 1); % Number of multipliers used for the convex hull

% Select state and input constraints matrices from polyhedrons
Hx  = X.A;  bx  = X.b;
Hu  = U.A;  bu  = U.b;
He  = invariantSet.A;  be  = invariantSet.b;

% ======= Constraints Definition ======

% Initial Condition
if isempty(bar_x_stored)
    Constraints = [He*(x_t-x{1})<=be];
else
    Constraints = [He*(x_t-x{1})<=be
                    x{1} == A*bar_x_stored + B*v;
                    v'*R*v <= bar_u_stored'*R*bar_u_stored];
end
% System Dynamics
for i = 1:N
    Constraints=[Constraints;
                 x{i+1} == A*x{i} + B*u{i};
                 Hx * x{i} <= bx;
                 Hu * u{i} <= bu;];
end

% Terminal Constraint: enforce predicted state in the convex safe set
Constraints=[Constraints;
             lambda >= 0;                        % Constraint the multipliers to be positive
             x{N+1} == SS*lambda;                % Terminal point in the convex hull
             ones(1,length(Qfun))*lambda == 1];  % Must be convex combination --> sum to 1


% ======= Cost Definition ======
% Running Cost
Cost=0;
for i=1:N
    Cost = Cost + x{i}'*Q*x{i} + u{i}'*R*u{i};
end 

% Terminal cost as convex combination of stored cost-to-go
Cost = Cost + Qfun*lambda;

% Solve the FTOCP
options = sdpsettings('verbose',0,'solver',solver);
options.OptimalityTolerance = 1e-15;
options.StepTolerance = 1e-15;
Problem = optimize(Constraints,Cost,options);
Objective = double(Cost);
end

