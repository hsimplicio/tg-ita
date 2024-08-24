function events = events(initial_states, final_states, parameters, t0, tf, xad, iphase, workspace)
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

    % Define the events function
    events = [
        initial_states(1);
        initial_states(2);
        initial_states(3);
        initial_states(4);
        initial_states(5);
        final_states(1);
        final_states(2);
        final_states(3);
        final_states(4);
    ];
end