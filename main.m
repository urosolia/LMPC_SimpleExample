clc
clear all
close all

%%
% Define your system 
solver = 'quadprog'; % Options are 'gurobi' or 'quadprog';
[A, B, U, X, Q, R, N] = DefineSystem();

%% Load the first feasible solution
load('feasibleSolution')

% Plot the feasible solution
figure
plot(x_cl(1,:),x_cl(2,:),'-o')
h = legend('First Feasible Solution','interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);
%% ========================================================================
%  ======================= Now Run Learning MPC ===========================
%  ========================================================================
%% Initialize Safe Set and Q-funtion
SS   = x_cl;                          % Safe set vector: Vector collecting the state of the performed iterations
uSS   = u_cl;                         % Safe Input set vector: Vector collecting the input of the performed iterations
Qfun = ComputeCost(x_cl, u_cl, Q, R); % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost = Qfun(1);
%% Now run the LMPC
% Pick number of iterations to run
Iterations = 20;

% Run the LMPC
x0 = x_cl(:,1);
[ x_LMPC, u_LMPC, SS, uSS, Qfunction, IterationCost] = LMPC(x0, SS, uSS, Qfun, A, B, Q, R, ...
                                                           N, Iterations, X, U, IterationCost, solver);

%% Compute Optimal Solution
[ x_opt, u_opt ] = solve_CFTOCP( x0, 300, Q, R, A, B, X, U, solver);
optCost = ComputeCost(x_opt, u_opt, Q, R); % Q-function vector: Vector collecting the cost-to-go of the store states

%% Display Cost and Plot the Results
format long
clc
totSS = [];
for i = 1:(Iterations+1)
    totSS = [totSS, SS{i}];
    fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost(i)]);
end

figure()
hold on
a = plot(totSS(1,:), totSS(2,:), 'or');
b = plot(x_LMPC(1,:), x_LMPC(2,:), '-sb');
c = plot(x_opt(1,:), x_opt(2,:), '-k*');
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
h = legend([a, b, c], 'Sampled Safe Set', 'LMPC closed-loop', 'Optimal Solution');
set(h,'fontsize',15)

fprintf('Optimal cost:  %13.4f\n', optCost(1));
%%
save('WorkspaceAfterLMPC')