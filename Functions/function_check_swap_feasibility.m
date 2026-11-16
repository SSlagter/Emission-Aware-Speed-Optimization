function result = function_check_swap_feasibility( ...
    segment_energy_kWh, ...
    expected_swaps, ...
    battery_capacity_kWh, ...
    initial_SOC, ...
    minimum_SOC)

% FUNCTION_CHECK_SWAP_FEASIBILITY
%
% Determine whether a voyage can be completed with exactly the
% expected number of battery swaps.
%
% Assumptions:
% - One battery container onboard
% - Battery starts at initial_SOC
% - Each swap replaces the battery with a fully charged container
% - Swaps are possible after each segment except the final segment
% - All swapping locations are available
% - No recharging during sailing
%
% The function does not alter the optimized speed profile.
% It only evaluates battery feasibility afterward.

E = segment_energy_kWh(:);

N = numel(E);

assert(expected_swaps >= 0 && ...
       expected_swaps <= N-1 && ...
       mod(expected_swaps,1) == 0, ...
       'Invalid expected number of swaps.');

assert(all(isfinite(E)) && all(E >= 0), ...
       'Segment energy must be finite and nonnegative.');

% Available energy from a fully charged battery
usable_energy_kWh = ...
    battery_capacity_kWh * (1 - minimum_SOC);

% Available energy from the initial battery
initial_usable_energy_kWh = ...
    battery_capacity_kWh * (initial_SOC - minimum_SOC);

% Candidate swap positions
candidate_positions = 1:(N-1);

% Generate all combinations with the requested number of swaps
if expected_swaps == 0

    combinations = zeros(1,0);

else

    combinations = nchoosek( ...
        candidate_positions, expected_swaps);

end

result.feasible = false;
result.swap_after_leg = [];
result.energy_between_swaps_kWh = [];
result.number_of_swaps = expected_swaps;

% Evaluate each candidate configuration
for k = 1:size(combinations,1)

    swap_positions = combinations(k,:);

    % Include starting point and destination
    boundaries = [0, swap_positions, N];

    interval_energy = zeros(1, numel(boundaries)-1);

    feasible_configuration = true;

    for j = 1:(numel(boundaries)-1)

        first_leg = boundaries(j) + 1;
        last_leg = boundaries(j+1);

        interval_energy(j) = ...
            sum(E(first_leg:last_leg));

        if j == 1
            available_energy = initial_usable_energy_kWh;
        else
            available_energy = usable_energy_kWh;
        end

        if interval_energy(j) > available_energy + 1e-8
            feasible_configuration = false;
            break;
        end

    end

    if feasible_configuration

        result.feasible = true;
        result.swap_after_leg = swap_positions;
        result.energy_between_swaps_kWh = interval_energy;

        break;

    end

end

% Additional reporting
result.total_energy_kWh = sum(E);

result.maximum_available_energy_kWh = ...
    initial_usable_energy_kWh + ...
    expected_swaps * usable_energy_kWh;

end