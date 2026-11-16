% Speed_Planning
% Choose a travel direction and powertrain, then optimize speed through
% water on each route segment. The model calculates vessel resistance,
% propulsion and source energy, sailing time, and onboard emissions.
% Results are reported for each segment and for the complete journey.
%
% Where to change the use case:
%   a. parameter_initialisation_ship                                        Vessel geometry and resistance inputs
%   b. parameter_initialisation_journey                                     Upstream route and water conditions
%   c. downstream (below)                                                   Direction of travel
%   d. parameter_initialisation_constraints                                 Deadlines, speed limits, and clearance
%   e. P_Hotel, P_calib, min_water_depth                                    Power and depth assumptions
%   f. parameter_initialisation_emissions                                   ICE engine class and emission factors
%   g. parameter_initialisation_powertrain                                  Powertrain architecture and curves
%   h. function_optimization                                                eta_D{z} relationship for the specific use case vessel
%   
% Journey inputs are ordered upstream in their initializer. For downstream
% travel, the direction function reverses the segments and changes the
% current sign before speed bounds and deadlines are calculated.
%
% FC-electric and BAT-electric assume energy is always available: this
% model does not track fuel inventory, battery SOC, or recharging.
% Electric cases report zero onboard operational emissions.

clear all

%% Initialise the voyage planning problem

% USER INPUTS — Battery swapping
expected_swaps = 3;            % Number of expected battery swaps
swap_duration_h = 0.5;         % Duration per swap [h]
number_of_containers = 2;      % Number of battery containers onboard
battery_capacity_kWh = 2900 * number_of_containers;   % ZES container nominal capacity * amount of containers
initial_SOC = 1.0;             % Initial battery SoC
minimum_SOC = 0.20;            % Minimum allowable battery SoC


% 1. Vessel parameters
% Vessel geometry, derived resistance coefficients, and transmission data
run parameter_initialisation_ship
ship_param = [L, T, T_F, B, h_B, Delta, S, S_B, S_app, A_T, A_BT, P_B];
ship_param = [ship_param, C_WP, C_P, C_A, C_B];
ship_param = [ship_param, c1, c2, c3, c4, c5, c7, c8, c9, c11, c12, c13, c14, c15, c16, c17];
ship_param = [ship_param, m1, m3, lambda, form_factor_k1, form_factor_k2];
ship_param = [ship_param, eta_G, eta_S];

% 2. Journey parameters
% Route distance, segment-average depth and current, and water properties
run parameter_initialisation_journey
downstream = 0;    % USER INPUT: 0 = upstream; 1 = downstream
[U_c, d, w_d] = function_set_journey_direction( ...
    U_c, d, w_d, downstream);
journey_param = [U_c, d, w_d];

% 3. Emission parameters
% ICE fuel and onboard-emission factors
% Must run before powertrain initialization.
run parameter_initialisation_emissions

% 4. Powertrain parameters
% Select ICE, FC-electric, or BAT-electric and load component curves.
% Requires emission_param from the preceding initialization.
run parameter_initialisation_powertrain

% 5. Voyage constraints
% Arrival windows, speed bounds, and minimum under-keel clearance.
% Must run after powertrain initialization because battery swapping
% time depends on the selected architecture.
run parameter_initialisation_constraints
constraint_param = [Tmax, Tmin, ub, lb, safety_margin];


% 6. Additional use-case parameters
P_Hotel = 0;           % Hotel demand while sailing [kW]
P_calib = 150;         % Calibrated additional propeller power [kW]
min_water_depth = 3.5; % Depth floor in calibrated eta_D relationship [m]

add_param = [N, vis, g, rho, P_Hotel, P_calib, min_water_depth];


%% Optimize the voyage
% Choose one speed-through-water setpoint per segment to minimize energy
% at the selected powertrain's source. The objective is engine output
% energy for ICE, hydrogen input energy for FC-electric, or battery
% chemical energy for BAT-electric. Arrival time and under-keel clearance
% are constrained; the wrapper reports solver runtime separately.

optimization_timer = tic;

[x_opt, fval, exitflag, information, ~, tt, results] = ...
    function_optimization(ship_param, journey_param, constraint_param, ...
                          add_param, emission_param, powertrain);

optimization_time_s = toc(optimization_timer);
%% Check information on optimization
% Energy_kWh has a different source basis for each architecture.
switch results.architecture
    case "ICE"
        energy_description = "engine output energy";
    case "FC-electric"
        energy_description = "hydrogen input energy";
    case "BAT-electric"
        energy_description = "battery chemical energy";
end

% Results by voyage segment
speed_water_kmh = x_opt(:) * 3.6;
speed_ground_kmh = (x_opt(:) + U_c(:)) * 3.6;
segment_time_h = d(:) ./ (x_opt(:) + U_c(:)) / 3600;

segment_results = table( ...
    (1:N)', d(:)/1000, speed_water_kmh, speed_ground_kmh, ...
    segment_time_h, results.clearance_m, ...
    results.source_power_kW, results.energy_kWh, ...
    results.fuel_L, results.H2_kg, ...
    results.CO2_kg, results.PM10_kg, results.NOx_kg, ...
    'VariableNames', {'Segment', 'Distance_km', ...
    'SpeedWater_kmh', 'SpeedGround_kmh', 'Time_h', ...
    'Clearance_m', 'SourcePower_kW', 'Energy_kWh', ...
    'Fuel_L', 'H2_kg', 'CO2_kg', 'PM10_kg', 'NOx_kg'});

fprintf('\n=== Results by segment: %s ===\n', ...
    char(results.architecture));
fprintf('Energy_kWh: %s.\n', char(energy_description));
disp(segment_results);

%% Journey totals
% Account for swapping time for battery-electric propulsion.
% The optimization already deducts this time from the sailing budget.

if results.architecture == "BAT-electric"
    actual_swap_time_h = expected_swaps * swap_duration_h;
else
    actual_swap_time_h = 0;
end

% Total voyage duration
total_voyage_time_h = tt + actual_swap_time_h;

% Journey results
journey_results = table( ...
    sum(d)/1000, ...
    tt, ...
    actual_swap_time_h, ...
    total_voyage_time_h, ...
    results.total_energy_kWh, ...
    results.total_fuel_L, ...
    results.total_H2_kg, ...
    results.total_CO2_kg, ...
    results.total_PM10_kg, ...
    results.total_NOx_kg, ...
    'VariableNames', { ...
    'Distance_km', ...
    'SailingTime_h', ...
    'SwappingTime_h', ...
    'VoyageTime_h', ...
    'Energy_kWh', ...
    'Fuel_L', ...
    'H2_kg', ...
    'CO2_kg', ...
    'PM10_kg', ...
    'NOx_kg'});

fprintf('\n=== Journey totals ===\n');
disp(journey_results);

%% Battery swapping feasibility

if results.architecture == "BAT-electric"

    swap_check = function_check_swap_feasibility( ...
        results.energy_kWh, ...
        expected_swaps, ...
        battery_capacity_kWh, ...
        initial_SOC, ...
        minimum_SOC);

    fprintf('\n=== Battery swapping feasibility ===\n');

    fprintf('Expected swaps: %d\n', expected_swaps);

    fprintf('Total battery energy: %.2f kWh\n', ...
        swap_check.total_energy_kWh);

    fprintf('Maximum available energy: %.2f kWh\n', ...
        swap_check.maximum_available_energy_kWh);

    fprintf('Swapping feasible: %d\n', ...
        swap_check.feasible);

    if swap_check.feasible

        fprintf('Feasible swap positions (after legs):\n');

        disp(swap_check.swap_after_leg);

        fprintf('Energy between swaps [kWh]:\n');

        disp(swap_check.energy_between_swaps_kWh);

    else

        fprintf(['No feasible swapping configuration found ', ...
            'for the requested number of swaps.\n']);

    end

end

%% Optimization result
% A positive exitflag means the solver reported convergence.
optimization_results = table( ...
    optimization_time_s, exitflag > 0, exitflag, fval, ...
    'VariableNames', {'Runtime_s', 'SolverConverged', ...
    'Exitflag', 'Objective_kWh'});

fprintf('\n=== Optimization ===\n');
fprintf('Objective_kWh: %s.\n', char(energy_description));
disp(optimization_results);

%% Detailed journey energy accounting
% Report journey totals only. Propeller energy already includes P_calib.
% Hotel energy is shown because it joins the power chain separately.

stage = results.stage;

stage_names = fieldnames(stage);

stage_energy_kWh = zeros(numel(stage_names), 1);

for k = 1:numel(stage_names)
    stage_energy_kWh(k) = sum(stage.(stage_names{k}));
end

stage_totals = table( ...
    string(stage_names), stage_energy_kWh, ...
    'VariableNames', {'EnergyStage', 'JourneyEnergy_kWh'});

fprintf('\n=== Journey energy at each stage ===\n');
disp(stage_totals);

%% Journey-average conversion efficiencies
% Each percentage is the ratio of total output to total input energy.
% The propulsive-efficiency row uses resistance-based propeller energy:
% the calibration was added after eta_D and is excluded from this ratio.

if results.architecture == "ICE"
    component = [ ...
        "Engine: diesel to output"
        "Mechanical transmission"
        "Propulsive efficiency"];

    input_kWh = [ ...
        sum(stage.diesel_chemical_kWh)
        sum(stage.engine_propulsion_kWh)
        sum(stage.resistance_propeller_kWh)];

    output_kWh = [ ...
        sum(stage.engine_output_kWh)
        sum(stage.propeller_kWh)
        sum(stage.effective_kWh)];

elseif results.architecture == "FC-electric"
    component = [ ...
        "Fuel cell"
        "Propulsion inverter"
        "Motor"
        "Mechanical transmission"
        "Propulsive efficiency"];

    input_kWh = [ ...
        sum(stage.hydrogen_chemical_kWh)
        sum(stage.propulsion_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.resistance_propeller_kWh)];

    output_kWh = [ ...
        sum(stage.total_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.propeller_kWh)
        sum(stage.effective_kWh)];

elseif results.architecture == "BAT-electric"

    component = [
        "Battery internal"
        "Battery-side DC/DC converter"
        "Propulsion inverter"
        "Motor"
        "Mechanical transmission"
        "Resistance-component efficiency"
        "Overall effective/propeller ratio"
    ];

    input_kWh = [
        sum(stage.battery_chemical_kWh)
        sum(stage.battery_terminal_kWh)
        sum(stage.propulsion_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.resistance_propeller_kWh)
        sum(stage.propeller_kWh)
    ];

    output_kWh = [
        sum(stage.battery_terminal_kWh)
        sum(stage.total_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.propeller_kWh)
        sum(stage.effective_kWh)
        sum(stage.effective_kWh)
    ];
end

efficiency_pct = 100 * output_kWh ./ input_kWh;
loss_kWh = input_kWh - output_kWh;

efficiency_results = table( ...
    component, input_kWh, output_kWh, loss_kWh, efficiency_pct, ...
    'VariableNames', {'Component', 'Input_kWh', 'Output_kWh', ...
    'Loss_kWh', 'JourneyEfficiency_pct'});

fprintf('\n=== Journey-average conversion efficiencies ===\n');
disp(efficiency_results);
if results.architecture == "BAT-electric"

    E_resistance = sum(stage.resistance_propeller_kWh);
    E_calibration = sum(stage.calibration_propulsion_kWh);
    E_propeller = sum(stage.propeller_kWh);
    E_effective = sum(stage.effective_kWh);

    fprintf('\n=== Propulsive energy decomposition ===\n');

    fprintf('Resistance-based contribution: %.2f kWh\n', ...
        E_resistance);

    fprintf('Additive calibration contribution: %.2f kWh\n', ...
        E_calibration);

    fprintf('Total propeller-delivered energy: %.2f kWh\n', ...
        E_propeller);

    fprintf('Effective towing energy: %.2f kWh\n', ...
        E_effective);

    fprintf('Resistance-component efficiency: %.2f %%\n', ...
        100 * E_effective / E_resistance);

    fprintf('Overall effective/propeller ratio: %.2f %%\n', ...
        100 * E_effective / E_propeller);

    fprintf('Energy balance residual: %.6f kWh\n', ...
        E_propeller - E_resistance - E_calibration);

end










%% ================================================================
%  Export voyage planning results to CSV
%  PATH2ZERO - M8 vessel
%
%  One row per route segment.
%  Output folder: Results
%
%  Includes:
%    - Scenario identification
%    - Route and environmental conditions
%    - Optimized speeds and sailing times
%    - Battery swapping feasibility
%    - Source power and energy
%    - Detailed powertrain energy stages
%    - Journey totals
% ================================================================


% 1. Scenario identification
vessel_name = "M8";

nodes = [ ...
    "Rotterdam"
    "Alblasserdam"
    "Gorinchem"
    "Tiel"
    "Nijmegen"
    "Emmerich"
    "Emmelsum"
    "Duisburg"];

if downstream == 0
    route_name = "Rotterdam-Duisburg";
else
    route_name = "Duisburg-Rotterdam";
    nodes = flipud(nodes);
end

architecture_name = string(results.architecture);


% 2. Battery swapping information
n_swaps = 0;
swap_time_total_h = 0;
swap_feasible = true;

swap_after_leg = zeros(N,1);

if results.architecture == "BAT-electric"

    n_swaps = expected_swaps;

    swap_time_total_h = ...
        expected_swaps * swap_duration_h;

    swap_feasible = swap_check.feasible;

    if swap_feasible
        swap_after_leg(swap_check.swap_after_leg) = 1;
    end

end


% 3. Segment speeds and sailing times
speed_water_mps = x_opt(:);

speed_ground_mps = ...
    x_opt(:) + U_c(:);

sailing_time_h = ...
    d(:) ./ speed_ground_mps / 3600;

cumulative_sailing_time_h = ...
    cumsum(sailing_time_h);


% 4. Arrival and departure times
arrival_time_h = NaN(N,1);
departure_time_h = NaN(N,1);

% Only construct actual stopping times if a feasible
% swapping configuration was identified.

if swap_feasible

    elapsed_time_h = 0;

    for z = 1:N

        elapsed_time_h = ...
            elapsed_time_h + sailing_time_h(z);

        arrival_time_h(z) = elapsed_time_h;

        if results.architecture == "BAT-electric"

            if swap_after_leg(z) == 1
                elapsed_time_h = ...
                    elapsed_time_h + swap_duration_h;
            end

        end

        departure_time_h(z) = elapsed_time_h;

    end

end


% 5. Create segment results table
export_results = table( ...
    repmat(vessel_name,N,1), ...
    repmat(route_name,N,1), ...
    repmat(architecture_name,N,1), ...
    repmat(journey_deadline_h,N,1), ...
    repmat(n_swaps,N,1), ...
    repmat(swap_time_total_h,N,1), ...
    repmat(swap_feasible,N,1), ...
    (1:N)', ...
    nodes(1:end-1), ...
    nodes(2:end), ...
    d(:)/1000, ...
    w_d(:), ...
    U_c(:), ...
    speed_water_mps, ...
    speed_ground_mps, ...
    speed_water_mps*3.6, ...
    speed_ground_mps*3.6, ...
    sailing_time_h, ...
    cumulative_sailing_time_h, ...
    arrival_time_h, ...
    departure_time_h, ...
    swap_after_leg, ...
    results.clearance_m(:), ...
    results.source_power_kW(:), ...
    results.energy_kWh(:), ...
    results.fuel_L(:), ...
    results.H2_kg(:), ...
    results.CO2_kg(:), ...
    results.PM10_kg(:), ...
    results.NOx_kg(:), ...
    'VariableNames', { ...
    'Vessel', ...
    'Route', ...
    'Powertrain', ...
    'Deadline_h', ...
    'SwapsExpected', ...
    'TotalSwapTime_h', ...
    'SwapFeasible', ...
    'Segment', ...
    'From', ...
    'To', ...
    'Distance_km', ...
    'WaterDepth_m', ...
    'Current_m_per_s', ...
    'SpeedWater_m_per_s', ...
    'SpeedGround_m_per_s', ...
    'SpeedWater_km_per_h', ...
    'SpeedGround_km_per_h', ...
    'SailingTime_h', ...
    'CumulativeSailingTime_h', ...
    'ArrivalTime_h', ...
    'DepartureTime_h', ...
    'SwapAfterLeg', ...
    'UnderKeelClearance_m', ...
    'SourcePower_kW', ...
    'Energy_kWh', ...
    'Fuel_L', ...
    'Hydrogen_kg', ...
    'CO2_kg', ...
    'PM10_kg', ...
    'NOx_kg'});


% 6. Add detailed powertrain energy accounting
% Export all energy stages already calculated in
% function_optimization.
%
% Only stages relevant to the selected architecture
% will be present.

stage_names = fieldnames(results.stage);

for k = 1:numel(stage_names)

    stage_name = stage_names{k};

    stage_values = results.stage.(stage_name);

    if isnumeric(stage_values) && ...
            numel(stage_values) == N

        column_name = matlab.lang.makeValidName( ...
            ['EnergyStage_' stage_name]);

        export_results.(column_name) = ...
            stage_values(:);

    end

end


% 7. Add journey totals
total_sailing_time_h = sum(sailing_time_h);

total_voyage_time_h = ...
    total_sailing_time_h + swap_time_total_h;

export_results.TotalDistance_km = ...
    repmat(sum(d)/1000,N,1);

export_results.TotalSailingTime_h = ...
    repmat(total_sailing_time_h,N,1);

export_results.TotalVoyageTime_h = ...
    repmat(total_voyage_time_h,N,1);

export_results.TotalEnergy_kWh = ...
    repmat(results.total_energy_kWh,N,1);

export_results.SolverConverged = ...
    repmat(exitflag > 0,N,1);

export_results.OptimizationRuntime_s = ...
    repmat(optimization_time_s,N,1);

% 8. Create Results folder
output_folder = fullfile(pwd, 'Results');

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end


% 9. Generate descriptive filename
% Example:
% PATH2ZERO_M8_Rotterdam-Duisburg_BAT-electric_3swaps_30h.csv

filename = sprintf( ...
    'PATH2ZERO_%s_%s_%s_%dswaps_%gh.csv', ...
    char(vessel_name), ...
    char(route_name), ...
    char(architecture_name), ...
    n_swaps, ...
    journey_deadline_h);

file_path = fullfile(output_folder, filename);


% 10. Export CSV
writetable(export_results, file_path);


% 11. Confirm export
fprintf('\n=== PATH2ZERO CSV export ===\n');

fprintf('File: %s\n', filename);
fprintf('Location: %s\n', file_path);

fprintf('Segments exported: %d\n', height(export_results));

fprintf('Total energy: %.2f kWh\n', ...
    results.total_energy_kWh);

fprintf('Expected swaps: %d\n', n_swaps);

fprintf('Total voyage time: %.2f h\n', ...
    total_voyage_time_h);

if results.architecture == "BAT-electric"
    fprintf('Swapping feasible: %d\n', swap_feasible);
end

fprintf('CSV export completed.\n');