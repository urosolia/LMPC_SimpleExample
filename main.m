clc
clear all
close all

%%
% Define your system 
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
Qfun = ComputeCost(x_cl, u_cl, Q, R); % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost = Qfun(1);
%% Now run the LMPC
% Pick number of iterations to run
Iterations = 5;

% Run the LMPC
x0 = x_cl(:,1);
[ x_LMPC, u_LMPC, SS, Qfunction, IterationCost] = LMPC(x0, SS, Qfun, A, B, Q, R, ...
                                                           N, Iterations, X, U, IterationCost);

save('Workspace')
%% Display Cost and Plot the Results
clc
for i = 1:(Iterations+1)
    fprintf('Iteration cost at iteration %d:  %13.5f\n', [i, IterationCost(i)]);
end

figure()
hold on
a = plot(SS(1,:), SS(2,:), 'or');
b = plot(x_LMPC(1,:), x_LMPC(2,:), '-sb');
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
h = legend([a, b], 'Sampled Safe Set', 'LMPC closed-loop');
set(h,'fontsize',15)