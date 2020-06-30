function [  ] = plotComparisonClosedLoop(  )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
clc
clear all
close all

load('data/LMPC_Example_2_N_4.mat')
x_LMPC_N_4 = x_LMPC;
u_LMPC_N_4 = u_LMPC;


load('data/LMPC_Example_2_N_3.mat')
x_LMPC_N_3 = x_LMPC;
u_LMPC_N_3 = u_LMPC;

%%
figure
hold on
a = plot(x_feasible(1,:), x_feasible(2,:), '-dm'); 
b = plot(x_LMPC_N_3(1,:), x_LMPC_N_3(2,:), '-or');
c = plot(x_LMPC_N_4(1,:), x_LMPC_N_4(2,:), '-sb');
d = plot(x_opt(1,:), x_opt(2,:), '--*k');
h = legend([a, b, c, d], 'First feasible trajectory', 'LMPC with $$N=3$$', 'LMPC with $$N=4$$', 'Optimal trajectory');
set(h,'fontsize',15, 'interpreter', 'latex')
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);

%%
figure
hold on
a = plot([0:size(u_LMPC_N_3,2)-1], u_LMPC_N_3, '-or'); 
b = plot([0:size(u_LMPC_N_4,2)-1], u_LMPC_N_4, '-sb');
c = plot([0:size(u_opt,2)-1],u_opt, '--*k'); 
d = plot([0, size(u_opt,2)-1],[1.5 1.5], '-k'); 
plot([0, size(u_opt,2)-1],[-1.5 -1.5], '-k'); 
h = legend([a, b, c, d], 'LMPC input for $$N=3$$', 'LMPC input for $$N=4$$', 'Optimal input', 'Input constraints') 
set(h,'fontsize',15, 'interpreter', 'latex')
xlabel('time step $$k$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$u_k$$', 'interpreter', 'latex','fontsize',20);
xlim([0,16])
ylim([-1.6,1.6])
end

