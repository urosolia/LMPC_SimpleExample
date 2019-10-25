function [ x_cl, u_cl ] = solve_CFTOCP( x_t , N, Q, R, A, B, X, U, solver)
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
u=sdpvar(size(B,2)*ones(1,N),ones(1,N));

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
Constraints =[Constraints;
                x{N+1} == [0;0]];
% ======= Cost Definition ======
% Running Cost
Cost=0;
for i=1:N        
    Cost = Cost + x{i}'*Q*x{i} + u{i}'*R*u{i};
end 
Cost = Cost + x{N+1}'*Q*x{N+1};

% Solve the FTOCP
options = sdpsettings('verbose',0,'solver',solver);
Problem = optimize(Constraints,Cost,options);
Objective = double(Cost);

x_cl = [];
for i = 1:(N+1)
    x_cl = [x_cl, double(x{i})];
end

u_cl = [];
for i = 1:(N)
    u_cl = [u_cl, double(u{i})];
end

end

