function resultado = fzero(a,b,vh,V,alpha)
    arguments
        a double
        b double
        vh double
        V double
        alpha double
    end

    tol = 1e-6;
    fa = viFun(a, vh, V, alpha);
    fb = viFun(b, vh, V, alpha);
    if fa * fb >= 0
        resultado = NaN;
        return;
    end 
    
    while (b - a) > tol
        c = (a + b) / 2;
        fc = viFun(c, vh, V, alpha);
        if fc == 0
            resultado = c;
            return;
        elseif fa * fc < 0
            b = c;
            % fb = fc;
        else
            a = c;
            fa = fc;
        end
    end

    resultado = (a + b) / 2;
end