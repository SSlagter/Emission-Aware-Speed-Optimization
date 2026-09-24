function [c, ceq] = function_NLconstraints( ...
    x, N, t_trip, Tmax, Tmin, w_d, safety_margin, T, squat)
% FUNCTION_NLCONSTRAINTS Check arrival times and under-keel clearance.
% fmincon requires every entry of c to be <= 0. There are no equality
% constraints. t_trip{z} is cumulative time through segment z.

    % Latest permitted arrival at the end of each segment
    for z = 1:N
        c(z) = t_trip{z}(x) - Tmax(z);
    end

    % Earliest permitted arrival at the end of each segment
    for z = 1:N
        c(z+N) = Tmin(z) - t_trip{z}(x);
    end

    % Require water depth - draught - squat >= safety margin
    for z = 1:N
        c(z+2*N) = T + safety_margin - w_d(z) + squat{z}(x);
    end

    ceq = [];
end