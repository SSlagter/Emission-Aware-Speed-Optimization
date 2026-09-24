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

% Vessel geometry, derived resistance coefficients, and transmission data
run parameter_initialisation_ship
ship_param = [L, T, T_F, B, h_B, Delta, S, S_B, S_app, A_T, A_BT, P_B];
ship_param = [ship_param, C_WP, C_P, C_A, C_B];
ship_param = [ship_param, c1, c2, c3, c4, c5, c7, c8, c9, c11, c12, c13, c14, c15, c16, c17];
ship_param = [ship_param, m1, m3, lambda, form_factor_k1, form_factor_k2];
ship_param = [ship_param, eta_G, eta_S];

% Route distance, segment-average depth and current, and water properties
run parameter_initialisation_journey
downstream = 0;                                                             % USER INPUT: 0 = upstream; 1 = downstream
[U_c, d, w_d] = function_set_journey_direction(U_c, d, w_d, downstream);
journey_param = [U_c, d, w_d];

% Arrival windows, speed bounds, and minimum under-keel clearance
% Calculate these after selecting direction because current affects bounds.
run parameter_initialisation_constraints
constraint_param = [Tmax, Tmin, ub, lb, safety_margin];

% Use-case assumptions; tune against vessel measurements where available.
P_Hotel = 50;                                                               % USER INPUT: hotel demand while sailing [kW]
P_calib = 150;                                                              % USER INPUT: extra propulsion power at propeller [kW]
min_water_depth = 3.5;                                                      % USER INPUT: depth floor in eta_D relationship [m]
add_param = [N, vis, g, rho, P_Hotel, P_calib, min_water_depth];

% ICE fuel and onboard-emission factors
run parameter_initialisation_emissions

% Select ICE, FC-electric, or BAT-electric and load component curves
run parameter_initialisation_powertrain

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

% Journey totals
journey_results = table( ...
    sum(d)/1000, tt, results.total_energy_kWh, ...
    results.total_fuel_L, results.total_H2_kg, ...
    results.total_CO2_kg, results.total_PM10_kg, ...
    results.total_NOx_kg, ...
    'VariableNames', {'Distance_km', 'TravelTime_h', ...
    'Energy_kWh', 'Fuel_L', 'H2_kg', ...
    'CO2_kg', 'PM10_kg', 'NOx_kg'});

fprintf('\n=== Journey totals ===\n');
disp(journey_results);

% Optimization result
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
stage_names = stage_names(~strcmp(stage_names, ...
    'calibration_propulsion_kWh'));

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
    component = [ ...
        "Battery internal"
        "Battery-side converter"
        "Propulsion inverter"
        "Motor"
        "Mechanical transmission"
        "Propulsive efficiency"];

    input_kWh = [ ...
        sum(stage.battery_chemical_kWh)
        sum(stage.battery_terminal_kWh)
        sum(stage.propulsion_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.resistance_propeller_kWh)];

    output_kWh = [ ...
        sum(stage.battery_terminal_kWh)
        sum(stage.total_DC_kWh)
        sum(stage.motor_electrical_kWh)
        sum(stage.motor_shaft_kWh)
        sum(stage.propeller_kWh)
        sum(stage.effective_kWh)];
end

efficiency_pct = 100 * output_kWh ./ input_kWh;
loss_kWh = input_kWh - output_kWh;

efficiency_results = table( ...
    component, input_kWh, output_kWh, loss_kWh, efficiency_pct, ...
    'VariableNames', {'Component', 'Input_kWh', 'Output_kWh', ...
    'Loss_kWh', 'JourneyEfficiency_pct'});

fprintf('\n=== Journey-average conversion efficiencies ===\n');
disp(efficiency_results);