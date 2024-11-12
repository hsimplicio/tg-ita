function [yInterp, dyInterp] = spline2(tOld, yOld, dyOld, tNew)
    % Second-order interpolation using the formula:
    % x(t) ≈ xk + fk*τ + (τ^2/2hk)*(fk+1 - fk)
    % where τ is local time and hk is the time step
    %
    % Inputs:
    %   tOld: old time vector [1, nOld]
    %   yOld: old state matrix [nStates, nOld]
    %   dyOld: old derivative matrix [nStates, nOld]
    %   tNew: new time vector [1, nNew]
    %
    % Outputs:
    %   yInterp: interpolated state matrix [nStates, nNew]
    %   dyInterp: interpolated derivative matrix [nStates, nNew]
    
    nStates = size(yOld, 1);
    yInterp = zeros(nStates, length(tNew));
    dyInterp = zeros(nStates, length(tNew));
    
    for i = 1:length(tNew)
        t = tNew(i);
        
        % Find the interval containing t
        idx = find(tOld(1:end-1) <= t & t <= tOld(2:end), 1);
        
        if isempty(idx)
            if t < tOld(1)
                idx = 1;
            else
                idx = length(tOld) - 1;
            end
        end
        
        % Get local time step
        hk = tOld(idx + 1) - tOld(idx);
        
        % Local time τ
        tau = t - tOld(idx);
        
        % Get values and derivatives (now matrices)
        xk = yOld(:,idx);
        fk = dyOld(:,idx);
        fk1 = dyOld(:,idx + 1);
        
        % Apply the formula (element-wise operations)
        yInterp(:,i) = xk + fk*tau + (tau^2/(2*hk))*(fk1 - fk);
        dyInterp(:,i) = fk + (tau/hk)*(fk1 - fk);
    end
end 