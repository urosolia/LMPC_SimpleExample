function [ x_cl,  u_cl] = computeClosedLoop( x0, bar_x_cl, v_cl, A, B, K, W )
%computeClosedLoop Computes noisi closed-loop trajectory
%   - x0: initial condition
%   - bar_x_cl: nominal trajectory
%   - v_cl: nominal input

x_cl = x0;
u_cl = 0;
for i = 1:(size(bar_x_cl,2)-1)
    w_t = sampledDisturbance( W );
    u_cl(i)   =-K*(x_cl(:, i) - bar_x_cl(:, i)) + v_cl(i);
    x_cl(:, i+1) = A*x_cl(:, i) + B*u_cl(:, i) + w_t;
end

end

