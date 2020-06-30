clc
clear all
close all

%% Options
LMPC_options.solver  = 'gurobi'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;
LMPC_options.norm    = 2;        % Options 1-norm or 2-norm;
LMPC_options.goalSet = 0;        % Options 0 => xf = origin, 1 xf = set around origin;

%% Pick example to run
% Define your system 
example = 2;
[A, B, U, X, Q, R] = DefineSystem(example);

% Set controller horizon
N = 4;
%%
% Compute Goal Set
if LMPC_options.goalSet == 1
    [ invariantGoalSet ] = computeInvariantGoalSet( A, B, Q, R );
    figure
    plot(invariantGoalSet,'wire',true)
else
    invariantGoalSet = [];
end
%% Load the first feasible solution
load('feasibleSolution')
if LMPC_options.goalSet == 1
    [K,~,~] = dlqr(A,B,Q,R);
    x_feasible = [x_feasible,    invariantGoalSet.V',x_feasible(:,end)];
    u_feasible = [u_feasible, -K*invariantGoalSet.V',u_feasible(end)];
end
% Plot the feasible solution
figure
plot(x_feasible(1,:),x_feasible(2,:),'-o')
h = legend('First Feasible Solution','interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);
%% ========================================================================
%  ======================= Now Run Learning MPC ===========================
%  ========================================================================
%% Initialize Safe Set and Q-funtion
x_cl{1} = x_feasible;                          % Safe set vector: Vector collecting the state of the performed iterations
u_cl{1} = u_feasible;                         % Safe Input set vector: Vector collecting the input of the performed iterations
Qfun = ComputeCost(x_feasible, u_feasible, Q, R, LMPC_options, invariantGoalSet, A, B); % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost{1} = Qfun;
%% Now run the LMPC
% Pick number of iterations to run
Iterations = 20;

% Run the LMPC
x0 = x_cl{1}(:,1);
[ x_LMPC, u_LMPC, x_cl, u_cl, IterationCost, SS] = LMPC(x0, x_cl, u_cl, IterationCost, A, B, Q, R, ...
                                                        N, Iterations, X, U, LMPC_options, invariantGoalSet);

%% Compute Optimal Solution
if LMPC_options.goalSet == 0
    [ x_opt, u_opt ] = solve_CFTOCP( x0, 300, Q, R, A, B, X, U, LMPC_options);
else
    [ x_opt, u_opt ] = solve_CFTOCP_goalSet( x0, 300, Q, R, A, B, X, U, LMPC_options, invariantGoalSet);
end
optCost = ComputeCost(x_opt, u_opt, Q, R, LMPC_options, invariantGoalSet, A, B ); % Q-function vector: Vector collecting the cost-to-go of the store states

%% Display Cost and Plot the Results
clc
SS = [];
for i = 1:(Iterations+1)
    SS = [SS, x_cl{i}];
    fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
end
fprintf('Optimal Cost:  %13.4f\n', [optCost(1)]);

%%
figure()
hold on
a  = plot(SS(1,:), SS(2,:), 'or');
aa  = plot(x_feasible(1,:), x_feasible(2,:), '-dm');
b  = plot(x_LMPC(1,:), x_LMPC(2,:), '-sb');
c  = plot(x_opt(1,:), x_opt(2,:), '--k*');
if LMPC_options.goalSet == 1
    plot(invariantGoalSet,'wire',true)
end
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
h = legend([a, aa, b, c], 'Stored data', 'First feaisble trajectory', 'LMPC closed-loop at convergence', 'Optimal solution');
set(h,'fontsize',16, 'interpreter', 'latex')

%%
itCost = [];
for i = 1:(Iterations+1)
    itCost = [itCost, IterationCost{i}(1)];
end
figure()
semilogy([1:size(itCost,2)],itCost, '-o')
hold on
semilogy([1,size(itCost,2)],[optCost(1), optCost(1)], '-o')

figure()
plot([1:size(itCost,2)],itCost, '-o')
hold on
plot([1,size(itCost,2)],[optCost(1), optCost(1)], '-o')

%%
start = Iterations+1;
figure()
hold on
for i = start:(Iterations+1)
    a = plot(u_cl{i}, '-ob');
end
b = plot(u_opt, '-*k');
h = legend([a, b], 'Input', 'Optimal Input');
set(h,'fontsize',15, 'interpreter', 'latex')

fprintf('Converged cost:  %13.4f\n', itCost(end));
fprintf('Optimal cost:  %13.4f\n', optCost(1));
%% Check if the Linear Independece Constraint Qualification Condition (LICQ) is satisfied
CS = Polyhedron(SS');
CS.computeVRep();
CS.computeHRep();

% Note that the LICQ has to be satified for horizon N - 1
if LMPC_options.goalSet == 0 
    lambdaMult = optimalityTestFunction( x_cl{end}, u_cl{end}, N - 1, X, U, A, B, Q, R );
else
    lambdaMult = []; % TO DO: need to compute gradient for distance objective
end

%%
save(['LMPC_Example_',num2str(example),'_N_',num2str(N)])