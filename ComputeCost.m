function [ Qfun] = ComputeCost( x,u,Q,R,LMPC_options, goalSet, A, B)
% ComputeCost 
% This function takes as inputs:
% - x = [x_0^j, ..., x_{T_j}^j]   : vector of stored 
%                                   states at the jth 
%                                   iteration 
% - u = [u_0^j, ..., u_{T_j-1}^j}]: vector of stored
%                                   inputs at the jth 
%                                   iteration
% - (Q,R): matrices defining the running cost 
%          h(x,u) = x^T Q x + u^T R u

if LMPC_options.goalSet == 1
    [K,~,~] = dlqr(A,B,Q,R);

    y = sdpvar(2,1);
    sdpvar x1 x2 v

    costOpt = ([x1;x2]-y)'*Q*([x1;x2]-y) + (v+K*y)'*R*(v+K*y);
    constr  = [goalSet.A*y <= goalSet.b;];
    ops = sdpsettings('verbose',0,'solver','gurobi');

    optimizeCostEvaluation = optimizer(constr,costOpt,ops,{x1, x2, v},{y, costOpt});
end

for i = 1:(size(x,2))
    Index = size(x,2)-i+1; % Need to start from the end
    if LMPC_options.goalSet == 1
        if i == 1
            if goalSet.A*x(:,Index) <= goalSet.b
                Cost = [0];
            else
                error = 1;
            end
        else
            sol = optimizeCostEvaluation{{x(1,Index),x(2,Index), u(:,Index)}};

            Cost = [Cost(1) + sol{2}, Cost];
        end
    else
        if LMPC_options.norm == 1
                if i == 1
                    Cost(Index) = sum(abs(Q*x(:,Index)));    
                else
                    Cost(Index) = Cost(Index+1) + sum(abs(Q*x(:,Index))) + abs(R*u(:,Index));    
                end
            else
                if i == 1
                    Cost(Index) = x(:,Index)'*Q*x(:,Index);    
                else
                    Cost(Index) = Cost(Index+1) + x(:,Index)'*Q*x(:,Index) + u(:,Index)'*R*u(:,Index);    
                end
        end
        
    end
Qfun = Cost;
end

