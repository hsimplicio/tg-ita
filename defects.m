function d = defects(dt, x, f)
    % This function computes the defects for
    % direct transcription using the Trapezoidal Rule.
    %
    % Inputs:
    %   dt: scalar = time step
    %   x: [n_state, n_grid] = matrix
    %   f: [n_state, n_grid] = matrix
    %
    % Outputs:
    %   d: [n_state, n_grid-1] = matrix of defects
    %

    n_grid = size(x, 2);

    index_low = 1:(n_grid-1);
    index_upp = 2:n_grid;

    x_low = x(:,index_low);
    x_upp = x(:,index_upp);

    f_low = f(:,index_low);
    f_upp = f(:,index_upp);

    % Apply the Trapezoidal Rule
    d = x_upp - x_low - 0.5 * dt * (f_low + f_upp);
end