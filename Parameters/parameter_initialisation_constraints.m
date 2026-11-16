%% Input parameters - voyage constraints
% Edit the deadlines, clearance margin, and speed limits in this section.
% U_c, d, and w_d must already be set for the selected travel direction.

% Arrival-time constraints [h]
% Tmax(z) and Tmin(z) apply to cumulative arrival time at the end of
% segment z. The same Tmax for every segment imposes one journey deadline.
% There are currently no minimum arrival times or intermediate windows.
Tmin = zeros(1, N);

%% Journey deadline

if downstream == 1
    journey_deadline_h = 18;   % Placeholder for downstream scenario
else
    journey_deadline_h = 30;
end

%% Account for battery swapping

if powertrain.architecture == "BAT-electric"

    total_swap_time_h = expected_swaps * swap_duration_h;

else

    total_swap_time_h = 0;

end

available_sailing_time_h = ...
    journey_deadline_h - total_swap_time_h;

assert(available_sailing_time_h > 0, ...
    'No sailing time remains after accounting for swaps.');

Tmax = ones(1,N) * available_sailing_time_h;

% Minimum water clearance after accounting for vessel squat.
safety_margin = 0.3;           % Required under-keel clearance [m]

% Speed limits. U_c is signed in the selected direction of travel.
% The optimizer chooses speed through water; speed over ground is
% speed through water plus U_c.
sow_min = 2;               % Minimum speed through water [m/s]
sow_max = 5;               % Maximum speed through water [m/s]
sog_min = 0.5;             % Minimum speed over ground [m/s]

lb = max(sow_min * ones(1, N), sog_min - U_c);
ub = sow_max * ones(1, N);

% Catch conflicting speed bounds before calling the optimizer.
assert(all(lb <= ub), ...
    'Speed bounds conflict on segment(s): %s', ...
    mat2str(find(lb > ub)));

% The shortest possible travel time under the speed bounds alone.
% Squat and other nonlinear constraints may require slower sailing.
fastest_time_h = sum(d ./ (ub + U_c)) / 3600;

assert(fastest_time_h <= available_sailing_time_h, ...
    'Deadline infeasible after accounting for swapping time.');