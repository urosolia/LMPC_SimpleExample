function [ x_LMPC, u_LMPC, bar_x_LMPC, bar_u_LMPC,x_bar_cl, u_bar_cl, IterationCost] = LMPC(x0, x_bar, u_bar,  IterationCost, A, B, Q, R, N, Iterations, X, U, solver, invariantSet, W)
%% Learning Model Predictive Control
% This function runs the LMPC for a user-defined number of iterations.
% The input to this functions are:
% - x0: initiol condition used at each iteration
% - x_bar: the initial feasible state sequence
% - u_bar: the initial feasible input sequence
% - IterationCost: cost associated with feasible trajectory
% - (A,B): matrices defining the system dynamics
% - (Q, R): cost matrices
% - N: LMPC horizon length
% - Iterations: max number of iterations
% - X: polyhedron representng the state constraints
% - U: polyhedron representng the input constraints
% - Solver: solver to use for the FTOCP
% - invariantSet: robust positive invariant for the error dynamics 
% - W: disturbance domain

% Now start iteration loop
j = 1;
x_bar_cl{1} = x_bar{1};
u_bar_cl{1} = u_bar{1};
[K,~,~] = dlqr(A,B,Q,R);

SS = x_bar{1};
Qfun = IterationCost{1};

while (j <= Iterations)
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
            v_old = bar_u_LMPC(:,t-1);
        else
            bar_x_old = [];
            v_old = [];            
        end
        [bar_x_Pred, vPred, vOld ] = FTOCP(x_LMPC(:,t), N, Q, R, Qfun, SS,...
                                                 A, B, X, U, solver, invariantSet, bar_x_old, v_old);
        
        % Update system position
        if t > 1
            bar_u_LMPC(:,t-1) = double(vOld);
        end

        bar_x_LMPC(:,t) = double(bar_x_Pred{1});
        checkState = bar_x_LMPC(:,t);
        bar_u_LMPC(:,t) = double(vPred{1});
        u_LMPC(:,t) = -K*(x_LMPC(:,t) - bar_x_LMPC(:,t)) + bar_u_LMPC(:,t);
        wt = sampledDisturbance(W);
        x_LMPC(:,t+1) = A*x_LMPC(:,t) + B*u_LMPC(:,t) + wt;
        


        t = t + 1;
    end

    % Now save the data, update cost and safe set.
    x_bar_cl{j+1} = bar_x_LMPC;
    u_bar_cl{j+1} = bar_u_LMPC;
    IterationCost{j+1} = ComputeCost(bar_x_LMPC, bar_u_LMPC, Q, R);
    SS   = [SS, bar_x_LMPC];
    Qfun = [Qfun, ComputeCost(bar_x_LMPC, bar_u_LMPC, Q, R)];
    totalCostVector = ComputeCost(bar_x_LMPC, bar_u_LMPC, Q, R);

    % increase Iteration index and restart
    j = j + 1;
    if j <= Iterations
        clear x_LMPC
        clear u_LMPC
    end
    
end
end