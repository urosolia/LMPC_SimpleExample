function [ x_LMPC, u_LMPC, SS, Qfun, IterationCost] = LMPC(x0, SS, Qfun, A, B, Q, R, N, Iterations, X, U, IterationCost)
%% Learning Model Predictive Control

% Now start iteration loop
j = 1;
while (j <= Iterations)    
    t = 1;        % Initialize time
    x_LMPC = x0;  % Initial condition

    % This is the time loop for the j-th iteration. This loop terminates when the
    % closed-loop trajectory reaches the terminal point x_F=0.
    while ( x_LMPC(:,t)'*x_LMPC(:,t) >(10^(-8)) )
        clc
        disp(['Time step: ', num2str(t), ', Iteration: ', num2str(j)])
        
        % Solve the LMPC at time t of the j-th iteration
        [xPred, uPred ] = FTOCP(x_LMPC(:,t), N, Q, R, Qfun, SS, A, B, X, U);

        % Update system position
        u_LMPC(:,t) = double(uPred{1});
        x_LMPC(:,t+1) = A*x_LMPC(:,t) + B*u_LMPC(:,t) ;

        t = t + 1;
    end
    
    % Now save the data, update cost and safe set.
    SS   = [SS, x_LMPC];
    Qfun = [Qfun, ComputeCost(x_LMPC, u_LMPC, Q, R)];
    totalCostVector = ComputeCost(x_LMPC, u_LMPC, Q, R);
    IterationCost(j+1) = totalCostVector(1);
    save('Workspace')

    % increase Iteration index and restart
    j = j + 1;
    if j <= Iterations
        clear x_LMPC
        clear u_LMPC
    end
    
end
end