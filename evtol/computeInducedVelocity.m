function vi = computeInducedVelocity(vh, V, alpha)
    % Computes induced velocity using momentum theory
    %
    % Inputs:
    %   vh: [1,n] hover induced velocity
    %   V: [1,n] free stream velocity
    %   alpha: [1,n] angle between free stream and rotor disk normal
    %
    % Outputs:
    %   vi: [1,n] induced velocity
    
    arguments
        vh double
        V double
        alpha double
    end
    
    % Input validation
    assert(all(isfinite(vh)), 'Non-finite hover velocity');
    assert(all(isfinite(V)), 'Non-finite velocity');
    assert(all(isfinite(alpha)), 'Non-finite angle');
    
    % For hover (V=0), vi should equal vh
    hover_condition = (V < 1e-6);

    vi = zeros(1,length(vh));
    vi(hover_condition) = vh(hover_condition);

    vi_func = @(w,vh,V,alpha) w - vh .^ 2 ./ (sqrt((V .* cos(alpha)) .^ 2) + (V .* sin(alpha) + w) .^ 2);

    % For non-hover cases, use fzero
    non_hover = ~hover_condition;
    options = optimset('TolX', 1e-6);

    for i = find(non_hover)
        vi(i) = fzero(@(w) vi_func(w,vh(i),V(i),alpha(i)), [-50,50], options);
    end
end