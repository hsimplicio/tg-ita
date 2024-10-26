function [c, ceq] = constraints(t, x, u, defects, bnd_cst, path_cst)
    % Define the collocation constraints
    % Inputs:
    %   t = [1,n] = time vector
    %   x = [5,n] = state variables
    %   u = [2,n] = control variables
    %   defects = defect constraints
    %   bnd_cst = boundary constraints
    %   path_cst = path constraints
    %
    % Outputs:
    %   c = [m,1] = inequality constraints
    %   ceq = [m,1] = equality constraints
    %

    % Initialize inequality and equality constraints
    c = [];
    ceq = [];
    
    % Evaluate defects constraints
    if ~isempty(defects)
        ceq = [ceq; defects(t, x, u)];
    end
    
    % Evaluate boundary constraints
    if ~isempty(bnd_cst)
        [c_bnd, ceq_bnd] = bnd_cst(x(:,1), x(:,end), t(1), t(end));
        c = [c; c_bnd];
        ceq = [ceq; ceq_bnd];
    end
    
    % Evaluate path constraints
    if ~isempty(path_cst)
        [c_path, ceq_path] = path_cst(t, x, u);
        c = [c; c_path];
        ceq = [ceq; ceq_path];
    end
end