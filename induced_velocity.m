function vi = induced_velocity(vh, V, alpha)
    % Compute the induced velocity
    %
    % INPUTS:
    % vh = [1,n] = induced velocity at hover
    % V = [1,n] = velocity magnitude
    % alpha = [1,n] = angle of attack
    %
    arguments
        vh double
        V double
        alpha double
    end

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