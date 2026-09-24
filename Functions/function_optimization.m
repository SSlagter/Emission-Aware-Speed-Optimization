function [out1, out2, out3, out4, out5, out6, out7] = ...
    function_optimization(ship_param, journey_param, constraint_param, ...
                          add_param, emission_param, powertrain)
% FUNCTION_OPTIMIZATION Find segment speeds that minimize voyage energy.
% For the selected ICE, FC-electric, or BAT-electric architecture, the
% function calculates resistance, power demand, source energy, travel
% time, and operational emissions. It applies speed, arrival-time, and
% under-keel-clearance constraints, then returns the optimized speeds,
% solver information, journey time, and segment-level results.
%
% Inputs are prepared by the parameter_initialisation_* scripts and
% packed in Voyage_Planning. out7 contains the results and energy
% accounting used by the reporting section.

%% Parameters
% Unpack vessel geometry, resistance coefficients, and transmission data.
convert = num2cell(ship_param);
[L, T, T_F, B, h_B, Delta, S, S_B, S_app, A_T, A_BT, P_B, C_WP, C_P, C_A, C_B, ...
    c1, c2, c3, c4, c5, c7, c8, c9, c11, c12, c13, c14, c15, c16, c17, ...
    m1, m3, lambda, form_factor_k1, form_factor_k2, eta_G, eta_S] = deal(convert{:});

% Unpack segment count, water properties, and use-case power settings.
convert = num2cell(add_param);
[N, vis, g, rho, P_Hotel, P_calib, min_water_depth] = deal(convert{:});

% Each journey vector contains N values in the selected travel direction.
[U_c, d, w_d] = deal(journey_param(1:N), journey_param(N+1:2*N), ...
    journey_param(2*N+1:3*N));

% Unpack cumulative arrival windows, speed bounds, and clearance margin.
[Tmax, Tmin, ub, lb, safety_margin] = deal(constraint_param(1:N), ...
    constraint_param(N+1:2*N), constraint_param(2*N+1:3*N), ...
    constraint_param(3*N+1:4*N), constraint_param(4*N+1));

% ICE fuel and emissions factors, including load-correction curves.
NOx_factor = emission_param.NOx_g_per_kWh;
PM10_factor = emission_param.PM10_g_per_kWh;
SFOC_base = emission_param.SFOC_base_g_per_kWh;
engine_rated_power = emission_param.rated_power_kW;
diesel_density = emission_param.diesel_density_g_per_L;
CO2_per_L = emission_param.CO2_kg_per_L;

load_percent = emission_param.load_percent(:);
NOx_correction = emission_param.NOx_correction(:);
PM10_correction = emission_param.PM10_correction(:);

% Select the power and energy calculation used in the segment loop.
architecture = powertrain.architecture;



%% Governing equations
for z = 1:N
    % Speed and sailing time on segment z
    V_ow{z} = @(x) x(z);                                  % Speed through water [m/s]
    t_k{z} = @(x) d(z) / ((V_ow{z}(x) + U_c(z)) * 3600); % Sailing time [h]

    % Reynolds numbers and friction coefficients
    Re{z} = @(x) L * V_ow{z}(x) / vis;

    if w_d(z)/T <= 4
        V1{z} = @(x) 0.4277 * V_ow{z}(x) * ...
            exp((w_d(z)/T)^(-0.07634));                   % Shallow-water adjusted speed
    else
        V1{z} = @(x) V_ow{z}(x);
    end

    Re1{z} = @(x) L * V1{z}(x) / vis;
    C_F_zero{z} = @(x) 0.075 / (log10(Re{z}(x)) - 2)^2;
    C_F_shallow{z} = @(x) ...
        (0.08169 / (log10(Re1{z}(x)) - 1.1717)^2) * ...
        (1 + (0.003998/(log10(Re1{z}(x)) - 4.393)) * ...
        ((w_d(z) - T)/L)^(-1.083));
    a_n{z} = @(x) 0.042612 * log10(Re{z}(x)) + 0.56725;
    C_F_katsui{z} = @(x) 0.0066577 / ...
        ((log10(Re{z}(x)) - 4.3762)^a_n{z}(x));

    if w_d(z)/T <= 4
        % Apply the Zeng shallow-water friction correction.
        C_F{z} = @(x) C_F_zero{z}(x) + ...
            (C_F_shallow{z}(x) - C_F_katsui{z}(x)) * ...
            (S_B/S) * (V1{z}(x)/V_ow{z}(x))^2;
    else
        C_F{z} = C_F_zero{z};                             % Deep-water friction
    end

    % Froude numbers and speed-dependent resistance coefficients
    F_n{z} = @(x) V_ow{z}(x)/sqrt(g*L);
    F_n2{z} = @(x) V_ow{z}(x)/sqrt(g*L);
    F_ni{z} = @(x) V_ow{z}(x) / ...
        sqrt(g*(T_F - h_B - 0.25*sqrt(A_BT)) + ...
        0.15*V_ow{z}(x)^2);
    F_nT{z} = @(x) V_ow{z}(x) / ...
        sqrt(2*g*A_T/(B + B*C_WP));

    c6{z} = @(x) function_c6(x, F_nT, z);
    m2{z} = @(x) c15*C_P^2*exp(-0.1*F_n2{z}(x)^(-2));
    m4{z} = @(x) c15*0.4*exp(-0.034*F_n2{z}(x)^(-3.29));

    % Hull, appendage, transom, correlation, and bulb resistances [N]
    R_F{z} = @(x) 0.5*rho*V1{z}(x)^2*S*C_F{z}(x);
    R_W{z} = @(x) function_RWSelection(x, c1, c2, c5, c17, ...
        F_n, Delta, rho, g, m1, m2, m3, m4, lambda, z);
    R_app{z} = @(x) 0.5*rho*V1{z}(x)^2*S_app * ...
        form_factor_k2*C_F{z}(x);
    R_tr{z} = @(x) 0.5*rho*V_ow{z}(x)^2*A_T*c6{z}(x);
    R_A{z} = @(x) 0.5*rho*V_ow{z}(x)^2*S*C_A;
    R_B{z} = @(x) 0.11*exp(-3*P_B^(-2))*F_ni{z}(x)^3 * ...
        A_BT^(1.5)*rho*g/(1 + F_ni{z}(x)^2);

    R_T{z} = @(x) R_F{z}(x)*form_factor_k1 + R_W{z}(x) + ...
        R_B{z}(x) + R_app{z}(x) + R_tr{z}(x) + R_A{z}(x);

    % Propulsion power: resistance demand plus calibrated propeller load.
    % Tune the depth-efficiency relationship for a different vessel.
    eta_D{z} = @(x) min(0.40 + 0.04 * ...
        (max(w_d(z), min_water_depth) - 3), 0.60);
    P_E{z} = @(x) R_T{z}(x)*V_ow{z}(x)/1000;       % Effective power [kW]
    P_resistance{z} = @(x) P_E{z}(x)/eta_D{z}(x);  % Resistance-based propeller power [kW]
    P_D{z} = @(x) P_resistance{z}(x) + P_calib;    % Total propeller power [kW]

    if architecture == "ICE"
        % Engine output supplies propulsion and hotel demand.
        P_source{z} = @(x) P_D{z}(x) / ...
            powertrain.ICE.mechanical_efficiency + P_Hotel;
        E{z} = @(x) P_source{z}(x)*t_k{z}(x);    % Engine output energy [kWh]

        engine_load{z} = @(x) min(max( ...
            P_source{z}(x)/engine_rated_power, 0), 1);
        SFOC{z} = @(x) (0.455*engine_load{z}(x)^2 - ...
            0.71*engine_load{z}(x) + 1.28)*SFOC_base;

        correction_load{z} = @(x) min(max( ...
            100*engine_load{z}(x), load_percent(1)), ...
            load_percent(end));

        fuel_L{z} = @(x) E{z}(x)*SFOC{z}(x)/diesel_density;
        CO2{z} = @(x) fuel_L{z}(x)*CO2_per_L;

        NOx{z} = @(x) E{z}(x)*NOx_factor * ...
            max(pchip(load_percent, NOx_correction, ...
            correction_load{z}(x)), 0)/1000;

        PM10{z} = @(x) E{z}(x)*PM10_factor * ...
            max(pchip(load_percent, PM10_correction, ...
            correction_load{z}(x)), 0)/1000;

        H2{z} = @(x) 0;

    else
        % Electric propulsion: mechanical input, motor, and DC-bus demand.
        % Curve inputs are limited to their tabulated ranges.
        P_motor{z} = @(x) P_D{z}(x) / ...
            powertrain.electric.mechanical_efficiency;

        motor_load{z} = @(x) min(max( ...
            100*P_motor{z}(x)/powertrain.motor.rated_kW, ...
            powertrain.motor.load_percent(1)), ...
            powertrain.motor.load_percent(end));

        eta_motor{z} = @(x) interp1( ...
            powertrain.motor.load_percent, ...
            powertrain.motor.efficiency, motor_load{z}(x), 'pchip');

        P_DC{z} = @(x) P_motor{z}(x) / ...
            (eta_motor{z}(x) * ...
            powertrain.electric.propulsion_inverter_efficiency) + ...
            P_Hotel;                                  % Total DC demand [kW]

        if architecture == "FC-electric"
            % Fuel-cell output is delivered directly to the DC bus.
            P_source{z} = @(x) P_DC{z}(x);

            FC_power{z} = @(x) min(max( ...
                P_source{z}(x), ...
                powertrain.FC.output_power_kW(1)), ...
                powertrain.FC.output_power_kW(end));

            eta_FC{z} = @(x) interp1( ...
                powertrain.FC.output_power_kW, ...
                powertrain.FC.efficiency, FC_power{z}(x), 'pchip');

            E{z} = @(x) P_source{z}(x)*t_k{z}(x) / ...
                eta_FC{z}(x);                         % Hydrogen input [kWh]
            H2{z} = @(x) E{z}(x) / ...
                powertrain.FC.hydrogen_LHV_kWh_per_kg;

        elseif architecture == "BAT-electric"
            % Battery terminal and internal losses are modeled separately.
            % Converter load is approximated using DC-bus demand.
            inverter_load{z} = @(x) min(max( ...
                P_DC{z}(x)/powertrain.inverter.rated_kW, ...
                powertrain.inverter.load_fraction(1)), ...
                powertrain.inverter.load_fraction(end));

            eta_inverter{z} = @(x) interp1( ...
                powertrain.inverter.load_fraction, ...
                powertrain.inverter.efficiency, ...
                inverter_load{z}(x), 'pchip');

            P_source{z} = @(x) P_DC{z}(x) / ...
                eta_inverter{z}(x);                    % Battery terminal [kW]

            battery_load{z} = @(x) min(max( ...
                P_source{z}(x)/powertrain.BAT.rated_kW, ...
                powertrain.BAT.load_fraction(1)), ...
                powertrain.BAT.load_fraction(end));

            eta_BAT{z} = @(x) interp1( ...
                powertrain.BAT.load_fraction, ...
                powertrain.BAT.efficiency, ...
                battery_load{z}(x), 'pchip');

            E{z} = @(x) P_source{z}(x)*t_k{z}(x) / ...
                eta_BAT{z}(x);                        % Battery chemical [kWh]
            H2{z} = @(x) 0;
        end

        % No onboard combustion emissions in either electric case.
        fuel_L{z} = @(x) 0;
        CO2{z} = @(x) 0;
        NOx{z} = @(x) 0;
        PM10{z} = @(x) 0;
    end

    % Squat contributes to the under-keel-clearance constraint.
    squat{z} = @(x) function_squat(x, z, V_ow, C_B, T, B, w_d(z));

    % Accumulate energy and travel time through segment z.
    if z == 1
        E_trip{z} = @(x) E{z}(x);
        t_trip{z} = @(x) t_k{z}(x);
    else
        E_trip{z} = @(x) E{z}(x) + E_trip{z-1}(x);
        t_trip{z} = @(x) t_k{z}(x) + t_trip{z-1}(x);
    end
end

%% Objective function and constraints
% Minimize total source energy over all segments. Its meaning depends on
% the selected architecture: engine output, hydrogen input, or battery
% chemical energy.
OBJ = @(x) E_trip{N}(x);

% Enforce cumulative arrival windows and minimum under-keel clearance.
% Speed-through-water bounds are supplied separately as lb and ub.
nonlin = @(x) function_NLconstraints(x, N, t_trip, ...
    Tmax, Tmin, w_d, safety_margin, T, squat);

%% Optimization
% Solve from five starting points; ub is the initial point supplied to
% the local solver. A positive exitflag reports solver convergence.
options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
    "SpecifyConstraintGradient", false, ...
    "SpecifyObjectiveGradient", false, ...
    'HessianFcn', [], 'Display', 'off', ...
    'StepTolerance', 1e-8, 'MaxIterations', 2000);

problem = createOptimProblem('fmincon', ...
    'objective', OBJ, 'x0', ub, 'lb', lb, 'ub', ub, ...
    'nonlcon', nonlin, 'options', options);

ms = MultiStart;
[x_opt, fval, exitflag, information] = run(ms, problem, 5);

%% Output - used for reporting
if isempty(x_opt)
    error('Optimization returned no solution. Exitflag: %d', exitflag);
end

% Optimizer outputs and journey-level travel information
out1 = x_opt;                         % Optimized speed through water [m/s]
out2 = fval;                          % Minimum source-energy objective [kWh]
out3 = exitflag;                      % Solver exit flag
out4 = information;                   % MultiStart information
out5 = V_ow{1}(x_opt) + U_c(1);      % Speed over ground on segment 1 [m/s]
out6 = t_trip{N}(x_opt);             % Total sailing time [h]

% Segment results. Energy refers to engine output, hydrogen input, or
% battery chemical energy, according to the selected architecture.
out7.architecture = architecture;
out7.energy_kWh = zeros(N, 1);
out7.source_power_kW = zeros(N, 1);
out7.fuel_L = zeros(N, 1);
out7.H2_kg = zeros(N, 1);
out7.CO2_kg = zeros(N, 1);
out7.NOx_kg = zeros(N, 1);
out7.PM10_kg = zeros(N, 1);
out7.clearance_m = zeros(N, 1);

for z = 1:N
    out7.energy_kWh(z) = E{z}(x_opt);
    out7.source_power_kW(z) = P_source{z}(x_opt);
    out7.fuel_L(z) = fuel_L{z}(x_opt);
    out7.H2_kg(z) = H2{z}(x_opt);
    out7.CO2_kg(z) = CO2{z}(x_opt);
    out7.NOx_kg(z) = NOx{z}(x_opt);
    out7.PM10_kg(z) = PM10{z}(x_opt);
    out7.clearance_m(z) = w_d(z) - T - squat{z}(x_opt);
end

out7.total_energy_kWh = sum(out7.energy_kWh);
out7.total_fuel_L = sum(out7.fuel_L);
out7.total_H2_kg = sum(out7.H2_kg);
out7.total_CO2_kg = sum(out7.CO2_kg);
out7.total_NOx_kg = sum(out7.NOx_kg);
out7.total_PM10_kg = sum(out7.PM10_kg);

%% Energy accounting at each powertrain stage
% Values are per segment [kWh]. P_calib is included in propeller demand;
% hotel demand enters separately at the engine or DC bus.
out7.stage.effective_kWh = zeros(N, 1);
out7.stage.resistance_propeller_kWh = zeros(N, 1);
out7.stage.calibration_propulsion_kWh = zeros(N, 1);
out7.stage.propeller_kWh = zeros(N, 1);
out7.stage.hotel_kWh = zeros(N, 1);

out7.loss.propulsive_kWh = zeros(N, 1);

if architecture == "ICE"
    out7.stage.diesel_chemical_kWh = zeros(N, 1);
    out7.stage.engine_output_kWh = zeros(N, 1);
    out7.stage.engine_propulsion_kWh = zeros(N, 1);

    out7.loss.engine_conversion_kWh = zeros(N, 1);
    out7.loss.mechanical_kWh = zeros(N, 1);

else
    out7.stage.motor_shaft_kWh = zeros(N, 1);
    out7.stage.motor_electrical_kWh = zeros(N, 1);
    out7.stage.propulsion_DC_kWh = zeros(N, 1);
    out7.stage.total_DC_kWh = zeros(N, 1);

    out7.loss.mechanical_kWh = zeros(N, 1);
    out7.loss.motor_kWh = zeros(N, 1);
    out7.loss.propulsion_inverter_kWh = zeros(N, 1);

    if architecture == "FC-electric"
        out7.stage.hydrogen_chemical_kWh = zeros(N, 1);
        out7.loss.fuel_cell_kWh = zeros(N, 1);

    elseif architecture == "BAT-electric"
        out7.stage.battery_terminal_kWh = zeros(N, 1);
        out7.stage.battery_chemical_kWh = zeros(N, 1);

        out7.loss.battery_internal_kWh = zeros(N, 1);
        out7.loss.battery_converter_kWh = zeros(N, 1);
    end
end

for z = 1:N
    sailing_time = t_k{z}(x_opt);

    % Shared propulsion demand and hotel energy
    out7.stage.effective_kWh(z) = ...
        P_E{z}(x_opt) * sailing_time;

    out7.stage.resistance_propeller_kWh(z) = ...
        P_resistance{z}(x_opt) * sailing_time;

    out7.stage.calibration_propulsion_kWh(z) = ...
        P_calib * sailing_time;

    out7.stage.propeller_kWh(z) = ...
        P_D{z}(x_opt) * sailing_time;

    out7.stage.hotel_kWh(z) = ...
        P_Hotel * sailing_time;

    % The calibration is added after eta_D.
    out7.loss.propulsive_kWh(z) = ...
        out7.stage.resistance_propeller_kWh(z) - ...
        out7.stage.effective_kWh(z);

    if architecture == "ICE"
        % Diesel chemical energy -> engine output -> propeller
        out7.stage.engine_propulsion_kWh(z) = ...
            P_D{z}(x_opt) / ...
            powertrain.ICE.mechanical_efficiency * sailing_time;

        out7.stage.engine_output_kWh(z) = E{z}(x_opt);

        out7.stage.diesel_chemical_kWh(z) = ...
            out7.fuel_L(z) * diesel_density / 1000 * ...
            powertrain.ICE.diesel_LHV_kWh_per_kg;

        out7.loss.mechanical_kWh(z) = ...
            out7.stage.engine_propulsion_kWh(z) - ...
            out7.stage.propeller_kWh(z);

        out7.loss.engine_conversion_kWh(z) = ...
            out7.stage.diesel_chemical_kWh(z) - ...
            out7.stage.engine_output_kWh(z);

    else
        % DC-bus propulsion -> motor -> shaft -> propeller
        out7.stage.motor_shaft_kWh(z) = ...
            P_motor{z}(x_opt) * sailing_time;

        out7.stage.motor_electrical_kWh(z) = ...
            P_motor{z}(x_opt) / eta_motor{z}(x_opt) * ...
            sailing_time;

        out7.stage.propulsion_DC_kWh(z) = ...
            out7.stage.motor_electrical_kWh(z) / ...
            powertrain.electric.propulsion_inverter_efficiency;

        out7.stage.total_DC_kWh(z) = ...
            P_DC{z}(x_opt) * sailing_time;

        out7.loss.mechanical_kWh(z) = ...
            out7.stage.motor_shaft_kWh(z) - ...
            out7.stage.propeller_kWh(z);

        out7.loss.motor_kWh(z) = ...
            out7.stage.motor_electrical_kWh(z) - ...
            out7.stage.motor_shaft_kWh(z);

        out7.loss.propulsion_inverter_kWh(z) = ...
            out7.stage.propulsion_DC_kWh(z) - ...
            out7.stage.motor_electrical_kWh(z);

        if architecture == "FC-electric"
            % Hydrogen chemical energy -> fuel-cell DC output
            out7.stage.hydrogen_chemical_kWh(z) = E{z}(x_opt);

            out7.loss.fuel_cell_kWh(z) = ...
                out7.stage.hydrogen_chemical_kWh(z) - ...
                out7.stage.total_DC_kWh(z);

        elseif architecture == "BAT-electric"
            % Battery chemical energy -> terminal -> DC bus
            out7.stage.battery_terminal_kWh(z) = ...
                P_source{z}(x_opt) * sailing_time;

            out7.stage.battery_chemical_kWh(z) = E{z}(x_opt);

            out7.loss.battery_converter_kWh(z) = ...
                out7.stage.battery_terminal_kWh(z) - ...
                out7.stage.total_DC_kWh(z);

            out7.loss.battery_internal_kWh(z) = ...
                out7.stage.battery_chemical_kWh(z) - ...
                out7.stage.battery_terminal_kWh(z);
        end
    end
end