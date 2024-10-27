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

    vi_func = @(w,vh,V,alpha) w - vh .^ 2 ./ (sqrt((V .* cos(alpha)) .^ 2) + (V .* sin(alpha) + w) .^ 2);

    options = optimset('TolX', 1e-6);

    vi = zeros(1,length(vh));
    for i = 1:length(vh)
        vi(i) = fzero(@(w) vi_func(w,vh(i),V(i),alpha(i)), [-50,50], options);
    end
end