function [ x_LMPC, u_LMPC] = runLMPC_Iteration(x0, x_bar, u_bar,  IterationCost, A, B, Q, R, N, IterationToRun, X, U, solver, invariantSet, W)
%% Learning Model Predictive Control
% This function runs the LMPC for a user-defined number of iterations.
% The input to this functions are:
% - x_bar: cell of stored feasible trajectories
% - u_bar: cell of stored feasible inputs
% - IterationCost: cell of cost associated with the trajectories
% - (A,B): matrices defining the system dynamics
% - (Q, R): cost matrices
% - N: LMPC horizon length
% - IterationToRun: iteration to run
% - X: polyhedron representng the state constraints
% - U: polyhedron representng the input constraints
% - Solver: solver to use for the FTOCP
% - invariantSet: robust positive invariant for the error dynamics 
% - W: disturbance domain


% Now start iteration loop
[K,~,~] = dlqr(A,B,Q,R);

SS = [];
Qfun = [];

j = IterationToRun;
for i = 1:j
    SS = [SS, x_bar{i}];
    Qfun = [Qfun, IterationCost{i}];
end

SSQfun = Polyhedron([SS', Qfun']);
SSQfun.computeHRep()
SSQfun.computeVRep()
SS = SSQfun.V(:,1:2)';
Qfun = SSQfun.V(:, end)';
    
t = 1;        % Initialize time
x_LMPC = x0;  % Initial condition
checkState = x0; % Just initialization for the while loop

% This is the time loop for the j-th iteration. This loop terminates when the
% closed-loop trajectory reaches the terminal point x_F=0.
if strcmp(solver,'gurobi')
    tollerance = 10^(-9);
else
    tollerance = 10^(-6);
end
while ( checkState'*checkState >(tollerance) )
    clc
    disp(['Time step: ', num2str(t), ', Iteration: ', num2str(j), ' Best Cost: ',num2str(IterationCost{j}(1))])

    % Solve the LMPC at time t of the j-th iteration
    if t>1
        bar_x_old = bar_x_LMPC(:,t-1);
        v_old = v_LMPC(:,t-1);
    else
        bar_x_old = [];
        v_old = [];            
    end
    [bar_x_Pred, vPred, vOld ] = FTOCP(x_LMPC(:,t), N, Q, R, Qfun, SS,...
                                             A, B, X, U, solver, invariantSet, bar_x_old, v_old);

    % Update system position
    if t > 1
        v_LMPC(:,t-1) = double(vOld);
    end

    bar_x_LMPC(:,t) = double(bar_x_Pred{1});
    checkState = bar_x_LMPC(:,t);
    v_LMPC(:,t) = double(vPred{1});
    u_LMPC(:,t) = -K*(x_LMPC(:,t) - bar_x_LMPC(:,t)) + v_LMPC(:,t);
    wt = sampledDisturbance(W);
    x_LMPC(:,t+1) = A*x_LMPC(:,t) + B*u_LMPC(:,t) + wt;



    t = t + 1;
end
    
end