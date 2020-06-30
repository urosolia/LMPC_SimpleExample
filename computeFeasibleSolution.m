clc
clear all
close all

LMPC_options.solver  = 'gurobi'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;
LMPC_options.norm    = 2;        % Options 1-norm or 2-norm;
LMPC_options.goalSet = 1;        % Options 1-norm or 2-norm;

%%
% Define your system 
solver = 'quadprog'; % Options are 'gurobi' or 'quadprog';
[A, B, U, X, Q, R, N] = DefineSystem();

setUp = 2;
%% Load the first feasible solution
x_max = X.V(1,1);
if setUp == 1
    v_max = 0.1;
else
    v_max = 2.0;
end
X_feasible = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);

if setUp == 1
    x0 = [-15;0];
else
    x0 = [-14;2];
end
[ x_feasible, u_feasible ] = solve_CFTOCP( x0, 200, 100*Q, R, A, B, X_feasible, U, LMPC_options);

% Plot the feasible solution
figure
plot(x_feasible(1,:),x_feasible(2,:),'-o')
h = legend('First Feasible Solution','interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);
%%
save('feasibleSolution.mat','x_feasible','u_feasible')
