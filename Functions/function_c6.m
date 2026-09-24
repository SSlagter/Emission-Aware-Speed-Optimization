function out = function_c6(x, F_nT, z)
% FUNCTION_C6 Calculate the transom-stern resistance coefficient.
% F_nT{z}(x) is the transom Froude number on segment z. The coefficient
% decreases with F_nT and becomes zero at F_nT = 5.

    variable1 = F_nT{z}(x);

    if variable1 < 5
        out = 0.2*(1 - 0.2*variable1);
    else
        out = 0;
    end
end