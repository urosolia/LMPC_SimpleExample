clc
clear all
close all

%%
% Define your system 
solver = 'quadprog'; % Options are 'gurobi' or 'quadprog';
[A, B, U, X, Q, R, N, W] = DefineSystem();

%% Compute Invariant
[K,~,~] = dlqr(A,B,Q,R);
Acl = A - B*K;
[ invariantSet ] = computeInvariant( Acl, W );
figure
plot(invariantSet,'wire',true,'color','w')
%% Compute the first feasible solution

% Set tighter constraints
barX = X-invariantSet;
barU = U-(-K*invariantSet);
x_max = barX.V(1,1);
v_max = 0.1;
X_feasible = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);

x0 = [-x_max;0];
[ bar_x_cl, v_cl ] = solve_CFTOCP( x0, 200, 100*Q, R, A, B, X_feasible, barU, solver, invariantSet);
[ x_cl, u_cl]      = computeClosedLoop(x0, bar_x_cl, v_cl, A, B, K, W); 

%% Check if error evolution is contained in the invariant
checkFlag = 1;
for i = 1:size(bar_x_cl,2)
    et = bar_x_cl(:,i)- x_cl(:,i);
    if invariantSet.contains(et)~= 1
        checkFlag = 0;
    end
end
if checkFlag == 1
    disp(['Error evolution is contained in the invariant']);
end
%%
% Plot the feasible solution
figure
a = plot(bar_x_cl(1,:),bar_x_cl(2,:),'-o');
hold on
b = plot(x_cl(1,:), x_cl(2,:), 'sr');

minkoskiSum = x0 + invariantSet;
c = plot(minkoskiSum,'wire',true,'color','w');

h = legend([a,b,c],{'First Feasible Solution','Initial Point','$$\bar x_t \oplus \mathcal{E}$$'},'interpreter','latex');
xlabel('$$x_1$$','interpreter','latex','fontsize',20);
ylabel('$$x_2$$','interpreter','latex','fontsize',20);
set(h,'fontsize',15);

save('feasibleSolution.mat','bar_x_cl','v_cl', 'x_cl', 'u_cl','invariantSet','K')
