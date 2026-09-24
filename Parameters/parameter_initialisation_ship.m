%% Input parameters - vessel
% Edit the values in this section to describe the vessel.
% The remaining sections derive Holtrop-Mennen resistance parameters
% from these inputs; normally they should not be edited individually.

% Principal dimensions and hull coefficients: M8 class use case
B = 11.4;                  % Moulded breadth [m]
L = 110;                   % Waterline length [m]
L_pp = 110;                % Length between perpendiculars [m]
T = 2.75;                  % Mean draught [m]
T_F = T;                   % Draught at forward perpendicular [m]
T_A = T;                   % Draught at aft perpendicular [m]
C_B = 0.85;                % Block coefficient [-]

% Hull features and appendages
A_BT = 0;                  % Transverse bulb area [m^2]; zero for no bulb
h_B = 0.2 * T;             % Height of bulb-area centre above keel [m]
A_T = 0.2 * B * T;         % Immersed transom area [m^2]
C_stern = 0;               % Stern-shape parameter [-]
D = 1.6;                   % Propeller diameter [m]
Z = 4;                     % Number of propeller blades [-]
S_app_factor = 0.05;       % Appendage wetted area as a fraction of hull area

% Engine and mechanical transmission
P_max = 1120;              % Engine rated power [kW]
eta_G = 0.96;              % Gearing efficiency [-]
eta_S = 0.98;              % Shaft/transmission efficiency [-]

% Basic checks on the user-set geometry
assert(all([B, L, L_pp, T, T_F, T_A, D] > 0), ...
    'Vessel dimensions and propeller diameter must be positive.');
assert(C_B > 0 && C_B < 1 && A_BT >= 0, ...
    'Require 0 < C_B < 1 and A_BT >= 0.');
assert(T_F > h_B && L/B > 2, ...
    'Check bulb-centre height and hull length-to-breadth ratio.');

%% Derived vessel geometry
% Displacement volume and hull-form coefficients
Delta = C_B * L * B * T;   % Displacement volume [m^3]
C_M = 1.006 - 0.005*C_B^(-3.56);  % Midship-section coefficient [-]
C_WP = (1 + 2*C_B)/3;      % Waterplane-area coefficient [-]
C_P = C_B/C_M;             % Prismatic coefficient [-]

% Longitudinal centre of buoyancy, expressed as a percentage aft of
% the midpoint of L_pp. The value is derived here from C_P.
lcb = 19.4*C_P - 13.5;     % [% of L_pp]

L_R = L * (1 - C_P + (0.06*C_P*lcb)/(4*C_P - 1));
                             % Length of run [m]
P_B = 0.56 * sqrt(A_BT) / (T_F - 1.5*h_B);
                             % Bulb emergence parameter [-]

assert(1 - C_P - 0.0225*lcb > 0, ...
    'Hull coefficients give an invalid entrance-angle expression.');

i_E = 1 + 89*exp(-(L/B)^(0.80856) * (1 - C_WP)^(0.30484) * ...
    (1 - C_P - 0.0225*lcb)^(0.6367) * ...
    (L_R/B)^(0.34574) * (100*Delta/L^3)^(0.16302));
                             % Waterline entrance angle [degrees]

S = L * (2*T + B) * sqrt(C_M) * ...
    (0.453 + 0.4425*C_B - 0.2862*C_M ...
    - 0.003467*B/T + 0.3696*C_WP) + 2.38*A_BT/C_B;
                             % Hull wetted area [m^2]

S_app = S_app_factor*S;     % Appendage wetted area [m^2]
S_B = L*B;                 % Approximate flat-bottom area [m^2]

%% Resistance-related parameter calculations
% Holtrop-Mennen coefficients derived from the vessel geometry.
% Keep these expressions consistent with the selected resistance model.

if (B/L) <= 0.11
    c7 = 0.229577 * (B/L)^0.33333;
elseif B/L > 0.25
    c7 = 0.5 - 0.0625 * (L/B);
else
    c7 = B/L;
end

assert(i_E < 90, ...
    'Entrance angle must be below 90 degrees for coefficient c1.');

c1 = 2223105 * c7^(3.78613) * (T/B)^(1.07961) * ...
    (90 - i_E)^(-1.37565);

c3 = 0.56 * A_BT^(1.5) / ...
    (B*T*(0.31 * sqrt(A_BT) + T_F - h_B));
c2 = exp(-1.89 * sqrt(c3));
c5 = 1 - 0.8*A_T/(B*T*C_M);
c13 = 1 + 0.003*C_stern;

if (T_F/L) <= 0.04
    c4 = T_F/L;
else
    c4 = 0.04;
end

if (B/T_A) < 5
    c8 = B*S/(L*D*T_A);
else
    c8 = S*(7*B/T_A - 25)/(L*D*(B/T_A - 3));
end

if c8 < 28
    c9 = c8;
else
    c9 = 32 - 16/(c8 - 24);
end

if T_A/D < 2
    c11 = T_A/D;
else
    c11 = 0.0833333*(T_A/D)^3 + 1.33333;
end

if T/L > 0.05
    c12 = (T/L)^0.2228446;
elseif T/L < 0.02
    c12 = 0.479948;
else
    c12 = 48.2*(T/L - 0.02)^2.078 + 0.479948;
end

if (L^3/Delta) < 512
    c15 = -1.69385;
elseif (L^3/Delta) > 1727
    c15 = 0;
else
    c15 = -1.69385 + (L/Delta^(1/3) - 8)/2.36;
end

c14 = 1 + 0.0011*C_stern;

if C_P < 0.8
    c16 = 8.07981*C_P - 13.8673*C_P^2 + 6.984388*C_P^3;
else
    c16 = 1.73014 - 0.7067*C_P;
end

c17 = 6919.3 * C_M^(-1.3346) * ...
    (Delta/L^3)^2.00977 * (L/B - 2)^1.40692;

if (L/B) < 12
    lambda = 1.446*C_P - 0.03*L/B;
else
    lambda = 1.446*C_P - 0.36;
end

% Wave-resistance parameters and model-ship correlation allowance
m1 = 0.0140407*(L/T) - 1.75254*Delta^(1/3)/L ...
    - 4.79323*(B/L) - c16;
m3 = -7.2035*(B/L)^0.326869*(T/B)^0.605375;

C_A = 0.006*(L + 100)^(-0.16) - 0.00205 + ...
    0.003*sqrt(L/7.5)*C_B^4*c2*(0.04 - c4);
C_P1 = 1.45*C_P - 0.315 - 0.0225*lcb;

% Form factors include the leading 1: (1 + k2) and (1 + k1).
form_factor_k2 = 2.50;    % Appendage form factor (1 + k2) [-]
form_factor_k1 = 0.93 + 0.487*c14*(B/L)^1.068 * ...
    (T/L)^0.461 * (L/L_R)^0.122 * ...
    (L^3/Delta)^0.365 * (1 - C_P)^(-0.604);