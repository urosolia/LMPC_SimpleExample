clc
clear all
close all

%%
% Define your system 
solver = 'gurobi'; % Only available option is gurobi as we solve a SOCP;
[A, B, U, X, Q, R, N, W] = DefineSystem();

%% Load the first feasible solution
load('feasibleSolution')

% Plot the feasible solution
figure
a = plot(bar_x_cl(1,:),bar_x_cl(2,:),'-o');
hold on
b = plot(x_cl(1,:), x_cl(2,:), 'sr');

minkoskiSum = x_cl(:,1) + invariantSet;
c = plot(minkoskiSum,'wire',true,'color','w');

h = legend([a,b,c],{'First Feasible Solution','Initial Point','$$\bar x_t \oplus \mathcal{E}$$'},'interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);
%% ========================================================================
%  ======================= Now Run Learning MPC ===========================
%  ========================================================================
%% Initialize Safe Set and Q-funtion
x_bar_cl{1}   = bar_x_cl;                          % Safe set vector: Vector collecting the state of the performed iterations
u_bar_cl{1}   = v_cl;                             % Safe Input set vector: Vector collecting the input of the performed iterations
IterationCost{1} = ComputeCost(bar_x_cl, v_cl, Q, R); % Q-function vector: Vector collecting the cost-to-go of the stored states
%% Now run the LMPC
% Pick number of iterations to run
Iterations = 14;

% Run the LMPC
x0 = x_cl(:,1);
barX = X - invariantSet;
barU = U - (K*invariantSet);
[ x_LMPC, u_LMPC, bar_x_LMCP, v_LMPC, x_bar_cl, u_bar_cl, IterationCost] = LMPC(x0, x_bar_cl, u_bar_cl, IterationCost, A, B, Q, R, ...
                                                           N, Iterations, barX, barU, solver, invariantSet, W);

%% Compute Optimal Solution
[ x_opt, u_opt ] = solve_CFTOCP( x0, 300, Q, R, A, B, barX, barU, solver, invariantSet);
optCost = ComputeCost(x_opt, u_opt, Q, R); % Q-function vector: Vector collecting the cost-to-go of the store states

%% Display Cost and Plot the Results
format long
clc
SS = [];

for i = 1:(Iterations)
    SS = [SS, x_bar_cl{i}];
    fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
end

figure()
hold on
a = plot(SS(1,:), SS(2,:), 'or');
b = plot(x_bar_cl{Iterations+1}(1,:), x_bar_cl{Iterations+1}(2,:), '-sb');
c = plot(x_opt(1,:), x_opt(2,:), '-k*');
% d = plot(minkoskiSum,'wire',true,'color','w');

xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
h = legend([a, b, c], 'Sampled Safe Set', 'LMPC closed-loop', 'Optimal Solution');
set(h,'fontsize',15)

fprintf('Optimal cost:  %13.4f\n', optCost(1));
%% Cost comparison
figure
hold on
LMPC_cost = [];
for i =2:(Iterations+1)
    LMPC_cost = [LMPC_cost, IterationCost{i}(1)];
end
a = plot(LMPC_cost,'-o')
b = plot([0,14],[optCost(1),optCost(1)],'--k')

ylim([585,593])
xlabel('Iteration', 'interpreter', 'latex','fontsize',20);
ylabel('Iteration Cost', 'interpreter', 'latex','fontsize',20);
h = legend([a, b], 'Iteration Cost', 'Optimal Cost');
set(h,'fontsize',15)

save('WorkspaceAfterLMPC')