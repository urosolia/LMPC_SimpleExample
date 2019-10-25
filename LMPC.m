function [ x_LMPC, u_LMPC, SS_out, uSS_out, Qfun_out, IterationCost] = LMPC(x0, SS, uSS, Qfun, A, B, Q, R, N, Iterations, X, U, IterationCost, solver)
%% Learning Model Predictive Control
% This function runs the LMPC for a user-defined number of iterations.
% The input to this functions are:
% - x0: initiol condition used at each iteration
% - SS: the initial safe set constructed using a feasible trajectory
% - (A,B): matrices defining the system dynamics
% - N: LMPC horizon length
% - Iterations: max number of iterations
% - X: polyhedron representng the state constraints
% - U: polyhedron representng the input constraints
% - IterationCost: vector collecting the iteration cost
% - Solver: solver to use for the FTOCP

% Now start iteration loop
j = 1;
SS_out{1} = SS;
uSS_out{1} = uSS;
Qfun_out{1} = Qfun;
while (j <= Iterations)
    SSQfun = Polyhedron([SS', Qfun']);
    SSQfun.computeHRep()
    SSQfun.computeVRep()
    SS = SSQfun.V(:,1:2)';
    Qfun = SSQfun.V(:, end)';
    
    t = 1;        % Initialize time
    x_LMPC = x0;  % Initial condition

    % This is the time loop for the j-th iteration. This loop terminates when the
    % closed-loop trajectory reaches the terminal point x_F=0.
    if strcmp(solver,'gurobi')
        tollerance = 10^(-9);
    else
        tollerance = 10^(-6);
    end
    while ( x_LMPC(:,t)'*x_LMPC(:,t) >(tollerance) )
        clc
        disp(['Time step: ', num2str(t), ', Iteration: ', num2str(j), ' Best Cost: ',num2str(Qfun_out{j}(1))])
        
        % Solve the LMPC at time t of the j-th iteration
        [~, uPred ] = FTOCP(x_LMPC(:,t), N, Q, R, Qfun, SS,...
                                                 A, B, X, U, solver);
        
        % Update system position
        u_LMPC(:,t) = double(uPred{1});
        x_LMPC(:,t+1) = A*x_LMPC(:,t) + B*u_LMPC(:,t) ;

        t = t + 1;
    end

    % Now save the data, update cost and safe set.
    SS_out{j+1} = x_LMPC;
    uSS_out{j+1} = u_LMPC;
    Qfun_out{j+1} = ComputeCost(x_LMPC, u_LMPC, Q, R);
    SS   = [SS, x_LMPC];
    Qfun = [Qfun, ComputeCost(x_LMPC, u_LMPC, Q, R)];
    totalCostVector = ComputeCost(x_LMPC, u_LMPC, Q, R);
    %% Remove redundant vertices
    IterationCost(j+1) = totalCostVector(1);

    % increase Iteration index and restart
    j = j + 1;
    if j <= Iterations
        clear x_LMPC
        clear u_LMPC
    end
    
end
end