clc
clear all
close all

load('workspaceComparison.mat')
figure()
hold on
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
plotOinf = plot(Oinf, 'wire',true,'edgecolor','m');
plotK3MPC = plot(K3MPC, 'wire',true,'edgecolor','r');
plotK3LMPC_S1 = plot(K3LMPC_S1, 'wire',true,'edgecolor','b');
plotK3LMPC_S2 = plot(K3LMPC_S2, 'wire',true,'edgecolor','k');
plotCinfty = plot(Cinf, 'wire',true,'edgecolor','m','linestyle','--');

h = legend([plotOinf, plotK3MPC, plotK3LMPC_S1, plotK3LMPC_S2, plotCinfty], '$$\mathcal{O}_\infty$$', 'MPC RoA', 'LMPC RoA (Algorithm 1)', 'LMPC RoA (Approximated Algorithm 1)', '$$\mathcal{C}_\infty$$');
set(h,'fontsize',16, 'interpreter', 'latex')

ylim([-15.5, 15.5])
xlim([-15.1, 15.1])    

%%
% Save Oinfty
temp = atan2d(Oinf.V(:,2),Oinf.V(:,1));
[~,i] = sort(temp);
temp_v = Oinf.V(i,:);
temp_v = [temp_v;temp_v(1,:)];

ddd = table;
ddd.x = temp_v(:,1);
ddd.y = temp_v(:,2);
writetable(ddd, 'data_plot/Oinf_data.dat', 'Delimiter','space');

% Save K3MPC
temp = atan2d(K3MPC.V(:,2),K3MPC.V(:,1));
[~,i] = sort(temp);
temp_v = K3MPC.V(i,:);
temp_v = [temp_v;temp_v(1,:)];

ddd = table;
ddd.x = temp_v(:,1);
ddd.y = temp_v(:,2);
writetable(ddd, 'data_plot/K3MPC_data.dat', 'Delimiter','space');

% Save K3LMPC_S1
temp = atan2d(K3LMPC_S1.V(:,2),K3LMPC_S1.V(:,1));
[~,i] = sort(temp);
temp_v = K3LMPC_S1.V(i,:);
temp_v = [temp_v;temp_v(1,:)];

ddd = table;
ddd.x = temp_v(:,1);
ddd.y = temp_v(:,2);
writetable(ddd, 'data_plot/K3LMPC_S1_data.dat', 'Delimiter','space');

% Save K3LMPC_S2
temp = atan2d(K3LMPC_S2.V(:,2),K3LMPC_S2.V(:,1));
[~,i] = sort(temp);
temp_v = K3LMPC_S2.V(i,:);
temp_v = [temp_v;temp_v(1,:)];

ddd = table;
ddd.x = temp_v(:,1);
ddd.y = temp_v(:,2);
writetable(ddd, 'data_plot/K3LMPC_S2_data.dat', 'Delimiter','space');

% Save Cinf
temp = atan2d(Cinf.V(:,2),Cinf.V(:,1));
[~,i] = sort(temp);
temp_v = Cinf.V(i,:);
temp_v = [temp_v;temp_v(1,:)];

ddd = table;
ddd.x = temp_v(:,1);
ddd.y = temp_v(:,2);
writetable(ddd, 'data_plot/Cinf_data.dat', 'Delimiter','space');