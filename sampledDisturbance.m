function [ w ] = sampledDisturbance( W )
%sampledDisturbance Sample disturbance from W
%   - W: disturbance support
n = size(W.V,2);
w = zeros(n,1);
for i = 1:n
    wMin = min(W.V(:,1));
    wMax = max(W.V(:,1));
    w(i,1) = unifrnd(wMin,wMax);
end

end

