clc
clear all
close all

%%
% Define your system 
solver = 'quadprog'; % Options are 'gurobi' or 'quadprog';
[A, B, U, X, Q, R, N] = DefineSystem();

%% Load the first feasible solution
load('feasibleSolution')
x_max = X.V(1,1);
v_max = 0.1;
X_feasible = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);

x0 = [-x_max;0];
[ x_feasible, u_feasible ] = solve_CFTOCP( x0, 200, 100*Q, R, A, B, X_feasible, U, solver);

% Plot the feasible solution
figure
plot(x_feasible(1,:),x_feasible(2,:),'-o')
h = legend('First Feasible Solution','interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);

save('feasibleSolution.mat','x_feasible','u_feasible')
