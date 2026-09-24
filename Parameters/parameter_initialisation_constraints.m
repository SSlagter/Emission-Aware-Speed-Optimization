%% Input parameters - voyage constraints
% Edit the deadlines, clearance margin, and speed limits in this section.
% U_c, d, and w_d must already be set for the selected travel direction.

% Arrival-time constraints [h]
% Tmax(z) and Tmin(z) apply to cumulative arrival time at the end of
% segment z. The same Tmax for every segment imposes one journey deadline.
% There are currently no minimum arrival times or intermediate windows.
Tmin = zeros(1, N);

if downstream == 1
    journey_deadline_h = 13;    % Downstream journey deadline [h]
else
    journey_deadline_h = 22.5;  % Upstream journey deadline [h]
end

Tmax = ones(1, N) * journey_deadline_h;

% Minimum water clearance after accounting for vessel squat.
safety_margin = 0.3;           % Required under-keel clearance [m]

% Speed limits. U_c is signed in the selected direction of travel.
% The optimizer chooses speed through water; speed over ground is
% speed through water plus U_c.
sow_min = 7/3.6;               % Minimum speed through water [m/s]
sow_max = 19/3.6;              % Maximum speed through water [m/s]
sog_min = 5/3.6;               % Minimum speed over ground [m/s]

lb = max(sow_min * ones(1, N), sog_min - U_c);
ub = sow_max * ones(1, N);

% Catch conflicting speed bounds before calling the optimizer.
assert(all(lb <= ub), ...
    'Speed bounds conflict on segment(s): %s', ...
    mat2str(find(lb > ub)));

% The shortest possible travel time under the speed bounds alone.
% Squat and other nonlinear constraints may require slower sailing.
fastest_time_h = sum(d ./ (ub + U_c)) / 3600;

assert(fastest_time_h <= journey_deadline_h, ...
    'Deadline %.2f h is below the speed-limited minimum of %.2f h.', ...
    journey_deadline_h, fastest_time_h);