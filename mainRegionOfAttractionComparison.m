clc
clear all
close all
addpath('utils','-end')

%% Options
LMPC_options.solver  = 'gurobi'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;
LMPC_options.norm    = 2;        % Options 1-norm or 2-norm;
LMPC_options.goalSet = 0;        % Options 0 => xf = origin, 1 xf = set around origin;

%% Pick example to run
% Define your system 
example = 2;
%[A, B, U, X, Q, R] = DefineSystemNew();
[A, B, U, X, Q, R] = DefineSystem(example);
 % Define the system matrices

% Set controller horizon
N = 3;

%% Region of attraction standard MPC
[P,L,G] = dare(A,B,Q,R); 
Finf=-(B'*P*B+R)^(-1)*B'*P*A;
Acl = A+B*Finf;
Ptr = dlyap(Acl',Finf'*R*Finf+Q);
system = LTISystem('A',Acl);
Xtilde = Polyhedron('A',[eye(2);-eye(2);Finf;-Finf],'b',[X.V(1,1);X.V(1,1);X.V(1,1);X.V(1,1);U.V(1,1);U.V(1,1)]);
Oinf = system.invariantSet('X',Xtilde);
Af = Oinf.A;
bf = Oinf.b;

K3MPC = computeX0_N3(A,B,X,U,Af,bf);

figure
hold on
plotOinf = plot(Oinf, 'wire',true);
plotK3MPC = plot(K3MPC, 'wire',true,'edgecolor','r');
h = legend([plotOinf, plotK3MPC], '$$\mathcal{O}_\infty$$', 'Region of Attraction MPC');
set(h,'fontsize',16, 'interpreter', 'latex')
%%
Iterations = 30;
x_cl{1} = zeros(2,1);                          % Safe set vector: Vector collecting the state of the performed iterations
u_cl{1} = 0;                         % Safe Input set vector: Vector collecting the input of the performed iterations
Qfun = 0; % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost{1} = Qfun;
invariantGoalSet = [];
% Run the LMPC
[ x_LMPC_S1, u_LMPC_S1, x_cl_S1, u_cl_S1, IterationCost_S1, SS_1] = LMPC_itEnlCompVer(x_cl, u_cl, IterationCost, A, B, Q, R, ...
                                                        N, Iterations, X, U, LMPC_options, invariantGoalSet);
%%
[ x_LMPC_S2, u_LMPC_S2, x_cl_S2, u_cl_S2, IterationCost_S2, SS_2] = LMPC_itEnlCompDir(x_cl, u_cl, IterationCost, A, B, Q, R, ...
                                                        N, Iterations, X, U, LMPC_options, invariantGoalSet);

                                                    %% Display Cost and Plot the Results
clc
[CS_S1] = plotCS(x_cl_S1, IterationCost_S1);
%%
[CS_S2] = plotCS(x_cl_S2,IterationCost_S2);


%%
Af = CS_S1{end}.A;
bf = CS_S1{end}.b;
K3LMPC_S1 = computeX0_N3(A,B,X,U,Af,bf);

Af = CS_S2{end}.A;
bf = CS_S2{end}.b;
K3LMPC_S2 = computeX0_N3(A,B,X,U,Af,bf);
%%
Cinf = maxInvariant(A,B,X,U);
%%
figure()
hold on
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
plotOinf = plot(Oinf, 'wire',true,'edgecolor','m');
plotK3MPC = plot(K3MPC, 'wire',true,'edgecolor','r');
plotK3LMPC_S1 = plot(K3LMPC_S1, 'wire',true,'edgecolor','b');
plotK3LMPC_S2 = plot(K3LMPC_S2, 'wire',true,'edgecolor','k');
plotCinfty = plot(Cinf, 'wire',true,'edgecolor','g','linestyle','--');

h = legend([plotOinf, plotK3MPC, plotK3LMPC_S1, plotK3LMPC_S2], '$$\mathcal{O}_\infty$$', 'MPC RoA', 'LMPC RoA (Algorithm 1)', 'LMPC RoA (Approximated Algorithm 1)');
set(h,'fontsize',16, 'interpreter', 'latex')

ylim([-15.5, 15.5])
xlim([-15.1, 15.1])    
                                                
save('workspaceComparison')

%%
if example == 3
    figure()
    hold on
    xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
    ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
    Voinf = [Oinf.V(:,2), Oinf.V(:,1)]
    Oinf1 = Polyhedron(Voinf)
    pltK = plot(X, 'wire',true,'edgecolor','k', 'linestyle', '--');
    plotOinf = plot(Oinf1, 'wire',true,'edgecolor','m', 'linestyle', '-', 'linewidth', 3);
    Voinf = [K3MPC.V(:,2), K3MPC.V(:,1)]
    K3MPC1 = Polyhedron(Voinf)
    plotK3MPC = plot(K3MPC1, 'wire',true,'edgecolor','r', 'linestyle', '-', 'linewidth', 3);
    Voinf = [K3LMPC_S1.V(:,2), K3LMPC_S1.V(:,1)]
    K3LMPC_S11 = Polyhedron(Voinf)
    plotK3LMPC_S1 = plot(K3LMPC_S11, 'wire',true,'edgecolor','b', 'linestyle', '-', 'linewidth', 3);
    h = legend([plotOinf, plotK3MPC, plotK3LMPC_S1], '$$\mathcal{O}_\infty$$', 'MPC', 'LMPC');
    set(h,'fontsize',16, 'interpreter', 'latex')

    ylim([-5.5, 5.5])
    xlim([-5.5, 5.5])   
end