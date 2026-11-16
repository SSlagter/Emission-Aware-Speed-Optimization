function [c, ceq] = function_NLconstraints( ...
    x, N, t_trip, Tmax, Tmin, ...
    w_d, safety_margin, T, squat, ...
    power_constraints)

% FUNCTION_NLCONSTRAINTS
%
% Nonlinear constraints for voyage speed optimization.
%
% Constraints:
%   1. Maximum cumulative arrival time
%   2. Minimum cumulative arrival time
%   3. Minimum under-keel clearance
%   4. Maximum powertrain component powers
%
% All inequality constraints must satisfy:
%
%       c(x) <= 0
%
% There are no equality constraints.

%% 1. Arrival-time constraints

c_time_max = zeros(N,1);
c_time_min = zeros(N,1);

for z = 1:N

    % Latest permitted arrival
    c_time_max(z) = ...
        t_trip{z}(x) - Tmax(z);

    % Earliest permitted arrival
    c_time_min(z) = ...
        Tmin(z) - t_trip{z}(x);

end


%% 2. Under-keel-clearance constraints

c_clearance = zeros(N,1);

for z = 1:N

    % Required:
    %
    % water depth - draught - squat >= safety_margin

    c_clearance(z) = ...
        T + safety_margin - w_d(z) + squat{z}(x);

end


%% 3. Powertrain component constraints

% Every function handle represents:
%
% Actual component power - maximum allowed power
%
% Therefore, feasible operation requires:
%
% power_constraints{k}(x) <= 0

n_power_constraints = numel(power_constraints);

c_power = zeros(n_power_constraints,1);

for k = 1:n_power_constraints

    c_power(k) = power_constraints{k}(x);

end


%% 4. Combine all inequality constraints

c = [ ...
    c_time_max;
    c_time_min;
    c_clearance;
    c_power];


%% 5. Equality constraints

ceq = [];

end