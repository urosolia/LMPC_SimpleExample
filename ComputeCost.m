function [ Qfun] = ComputeCost( x,u,Q,R,LMPC_options)
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

for i = 1:(size(x,2))
    Index = size(x,2)-i+1; % Need to start from the end
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

