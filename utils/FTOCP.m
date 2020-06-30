function [ x, uPred ] = FTOCP( x_t , N, Q, R, Qfun, SS, A, B, X, U, LMPC_options)
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

% Define Yalmip Variables
x=sdpvar(size(A,2)*ones(1,N+1),ones(1,N+1));
u=sdpvar(size(B,2)*ones(1,N),ones(1,N));
lambda = sdpvar(length(Qfun), 1); % Number of multipliers used for the convex hull

% Select state and input constraints matrices from polyhedrons
Hx  = X.A;  bx  = X.b;
Hu  = U.A;  bu  = U.b;

% ======= Constraints Definition ======

% Initial Condition
Constraints = [x_t == x{1}];

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
    if LMPC_options.norm == 1
        Cost = Cost + norm(Q*x{i},1) + norm(R*u{i},1);
    else
        Cost = Cost + x{i}'*Q*x{i} + u{i}'*R*u{i};
    end
end 

% Terminal cost as convex combination of stored cost-to-go
Cost = Cost + Qfun*lambda;

% Solve the FTOCP
options = sdpsettings('verbose',0,'solver',LMPC_options.solver);
% options.OptimalityTolerance = 1e-15;
% options.StepTolerance = 1e-15;
Problem = optimize(Constraints,Cost,options);
Objective = double(Cost);
uPred = double(u{1});

end

