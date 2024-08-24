function total_energy = endpoint_cost(initial_states, final_states, parameters, t0, tf, xad, iphase, workspace)
    % Define the end point (Mayer) cost function
    arguments
        initial_states adouble
        final_states adouble
        parameters adouble
        t0 adouble
        tf adouble
        xad adouble
        iphase int
        workspace Workspace
    end

    % Define the end point cost function
    total_energy = final_states(5);
end