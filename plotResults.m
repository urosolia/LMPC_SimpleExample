clc
clear all
close all

load('WorkspaceAfterLMPC')

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

%% Display Cost and Plot the Results
format long
clc
totSS = [];

subplot(2,2,1)
plotFun(1, x_bar_cl, IterationCost, x_opt)
subplot(2,2,2)
plotFun(2, x_bar_cl, IterationCost, x_opt)
subplot(2,2,3)
plotFun(4, x_bar_cl, IterationCost, x_opt)
subplot(2,2,4)
plotFun(14, x_bar_cl, IterationCost, x_opt)
