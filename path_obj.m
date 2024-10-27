function cost = path_obj(t, x, u)
    % Example function, not used in the eVTOL problem
    % Define the path objective L(·) - integrand of Lagrange Term
    % for the path planning problem
    % Inputs:
    %   t = [1,n] = time vector
    %   x = [5,n] = state matrix
    %   u = [2,n] = control input matrix
    % Output:
    %   cost = [1,n] = computed cost
    %

    cost = zeros(1, length(t));
end