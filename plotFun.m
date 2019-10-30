function [ ] = plotFun( Iterations, SS, IterationCost, x_opt )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    totSS = [];
    for i = 1:(Iterations)
        totSS = [totSS, SS{i}];
        fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
    end
    hold on
    a = plot(totSS(1,:), totSS(2,:), 'or');
    b = plot(SS{Iterations+1}(1,:), SS{Iterations+1}(2,:), '-sb');
    c = plot(x_opt(1,:), x_opt(2,:), '-k*');
    % d = plot(minkoskiSum,'wire',true,'color','w');

    xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
    ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
    h = legend([a, b, c], 'Sampled Safe Set', strcat('LMPC closed-loop at itearation ',' ',num2str(Iterations+1)), 'Optimal Solution');
    set(h,'fontsize',15)

end

