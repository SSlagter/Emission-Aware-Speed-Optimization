function out = function_RWSelection( ...
    x, c1, c2, c5, c17, F_n, Delta, rho, g, ...
    m1, m2, m3, m4, lambda, z)
% FUNCTION_RWSELECTION Calculate wave-making resistance on segment z.
% Use the low-Froude expression below Fn = 0.4 and the high-Froude
% expression above Fn = 0.55. Interpolate between them in the
% transition range. The returned resistance is in newtons.

    out1 = c1 * c2 * c5 * Delta * rho * g * ...
        exp(m1*F_n{z}(x)^(-0.9) + ...
        m2{z}(x)*cos(lambda*F_n{z}(x)^(-2)));

    out2 = c17 * c2 * c5 * Delta * rho * g * ...
        exp(m3*F_n{z}(x)^(-0.9) + ...
        m4{z}(x)*cos(lambda*F_n{z}(x)^(-2)));

    if F_n{z}(x) < 0.4
        out = out1;
    elseif F_n{z}(x) > 0.55
        out = out2;
    else
        out = out1 + ...
            (10*F_n{z}(x) - 4)*(out2 - out1)/1.5;
    end
end