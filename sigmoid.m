function y = sigmoid(x)
    % Compute the sigmoid function
    %
    % INPUTS:
    % x = [1,n] = input
    %
    % OUTPUTS:
    % y = [1,n] = output
    %
    arguments
        x double
    end

    y = 1 ./ (1 + exp(-x));
end