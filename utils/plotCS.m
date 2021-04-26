function [CS] = plotCS(x_cl, IterationCost)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
SSn{1} = [x_cl{1}];
CS{1} = Polyhedron(SSn{1}');
for i = 2:(size(x_cl,2))
    SSn{i} = [SSn{i-1}, x_cl{i}];
    CS{i} = Polyhedron(SSn{i}');
    fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
end
%%
figure()
hold on
CS1 = plot(CS{1}, 'wire', true, 'edgecolor', 'g');
CS2 = plot(CS{2}, 'wire', true, 'edgecolor', 'k');
CS3 = plot(CS{3}, 'wire', true, 'edgecolor', 'r');
CS4 = plot(CS{size(x_cl,2)}, 'wire', true, 'edgecolor', 'b');
h = legend([CS1, CS2, CS3, CS4], '$$\mathcal{CS}^0=\{0\}$$', '$$\mathcal{CS}^1=$$conv$$(\cup_{j=0}^1\cup_{t=0}^\infty x_t^j)$$', '$$\mathcal{CS}^2=$$conv$$(\cup_{j=0}^2\cup_{t=0}^\infty x_t^j)$$', '$$\mathcal{CS}^{20}=$$conv$$(\cup_{j=0}^{20}\cup_{t=0}^\infty x_t^j)$$');
set(h,'fontsize',16, 'interpreter', 'latex')
ylim([-15.1, 15.1])
xlim([-15.1, 15.1])  
xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
end

