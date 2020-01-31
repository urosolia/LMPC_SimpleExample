clc
clear all
close all

%% Options
LMPC_options.solver = 'quadprog'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;

%%
% Define your system 
[A, B, U, X, Q, R, N] = DefineSystem();

%% Load the first feasible solution
load('feasibleSolution')

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
Qfun = ComputeCost(x_feasible, u_feasible, Q, R); % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost{1} = Qfun;
%% Now run the LMPC
% Pick number of iterations to run
Iterations = 20;

% Run the LMPC
x0 = x_cl{1}(:,1);
[ x_LMPC, u_LMPC, x_cl, u_cl, IterationCost] = LMPC(x0, x_cl, u_cl, IterationCost, A, B, Q, R, ...
                                                        N, Iterations, X, U, LMPC_options);

%% Compute Optimal Solution
[ x_opt, u_opt ] = solve_CFTOCP( x0, 300, Q, R, A, B, X, U, LMPC_options.solver);
optCost = ComputeCost(x_opt, u_opt, Q, R); % Q-function vector: Vector collecting the cost-to-go of the store states

%% Display Cost and Plot the Results
format long
clc
SS = [];
for i = 1:(Iterations+1)
    SS = [SS, x_cl{i}];
    fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
end

%%
figure()
hold on
a = plot(SS(1,:), SS(2,:), 'or');
b = plot(x_LMPC(1,:), x_LMPC(2,:), '-sb');
c = plot(x_opt(1,:), x_opt(2,:), '-k*');
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
h = legend([a, b, c], 'Sampled Safe Set', 'LMPC closed-loop', 'Optimal Solution');
set(h,'fontsize',15)

figure()
hold on
for i = 1:(Iterations+1)
    a = plot(u_cl{i}, '-ob');
end
b = plot(u_opt, '-*k');
h = legend([a, b], 'Input', 'Optimal Input');
set(h,'fontsize',15)

fprintf('Optimal cost:  %13.4f\n', optCost(1));
%%
save('WorkspaceAfterLMPC')