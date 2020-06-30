function [ x, uPred ] = FTOCP_goalSet( x_t , N, Q, R, Qfun, SS, A, B, X, U, LMPC_options, goalSet)
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

% Define Yalmip Variables
x=sdpvar(size(A,2)*ones(1,N+1),ones(1,N+1));
d=sdpvar(size(A,2)*ones(1,N+1),ones(1,N+1));

[K,~,~] = dlqr(A,B,Q,R);


if N == 1
    u{1}=sdpvar(size(B,2)*ones(1,N),ones(1,N));
else
    u=sdpvar(size(B,2)*ones(1,N),ones(1,N));
end
lambda = sdpvar(length(Qfun), 1); % Number of multipliers used for the convex hull

% Select state and input constraints matrices from polyhedrons
Hx  = X.A;  bx  = X.b;
Hu  = U.A;  bu  = U.b;
Ho  = goalSet.A;  bo  = goalSet.b;
% ======= Constraints Definition ======

% Initial Condition
Constraints = [x_t == x{1}];

% System Dynamics
for i = 1:N
    Constraints=[Constraints;
                 x{i+1} == (A-B*K)*x{i} + B*u{i};
                 Hx * x{i} <= bx;
                 Hu * (u{i}-K*x{i}) <= bu;
                 Ho * d{i} <= bo];
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
    Cost = Cost + (x{i}-d{i})'*Q*(x{i}-d{i}) + (u{i}-K*x{i}+K*d{i})'*R*(u{i}-K*x{i}+K*d{i});
end 

% Terminal cost as convex combination of stored cost-to-go
Cost = Cost + Qfun*lambda;

% Solve the FTOCP
options = sdpsettings('verbose',0,'solver',LMPC_options.solver);
Problem = optimize(Constraints,Cost, options);
Objective = double(Cost);

uPred = double(u{1})-K*double(x{1});
end

