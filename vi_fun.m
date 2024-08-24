function resultado = vi_fun(x, vh, V, alpha)
    % Find the induced velocity
    arguments
        x double
        vh double
        V double
        alpha double
    end

    % replace this with your equation
    resultado = x - vh * vh / (sqrt((V * cos(alpha)) * (V * cos(alpha)) + (V * sin(alpha) + x) * (V * sin(alpha) + x)));
end