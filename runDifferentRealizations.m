clc
clear all
close all
load('WorkspaceAfterLMPC')

%%
IterationToRun = 14;
distReal = 5;
x_cl_realization{1} = [];
u_cl_realization{1} = [];

for i = 1:distReal
    [ x_LMPC, u_LMPC] = runLMPC_Iteration(x0, x_bar_cl, u_bar_cl,  IterationCost, A, B, Q, R, N, IterationToRun, barX, barU, solver, invariantSet, W);
    x_cl_realization{i} = x_LMPC;
    u_cl_realization{i} = u_LMPC;
    
end

%%
figure 
hold on
for i = 1:distReal
    a = plot(x_cl_realization{i}(1,:), x_cl_realization{i}(2,:), '-ob');
    
end
b = plot(invariantSet,'wire',true,'color','w','edgecolor','r');
c = plot(X,'wire',true,'color','w','edgecolor','k');
d = plot(barX,'wire',true,'color','w','linestyle','--','edgecolor','k');

xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',24);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',24);
h = legend([a, b, c, d], 'Closed-loop', '$$\mathcal{E}$$', '$$\mathcal{X}$$', '$$\bar \mathcal{X}$$');
set(h,'fontsize',24, 'interpreter', 'latex')

%%
figure 
hold on
for i = 1:distReal
    a = plot(u_cl_realization{i}, '-ob');
    
end
%%
save('noisyRealizations')