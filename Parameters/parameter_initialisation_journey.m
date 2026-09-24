%% Input parameters - journey
% Route, in upstream travel order from Rotterdam to Duisburg.
% Each entry in d, w_d, and U_c describes the same route segment:
%
%   1  Rotterdam         -> Dordrecht          39 km
%   2  Dordrecht         -> Gorinchem          20 km
%   3  Gorinchem         -> Tiel               42 km
%   4  Tiel              -> Nijmegen-a         26 km
%   5  Nijmegen-a        -> Nijmegen-b         10 km
%   6  Nijmegen-b        -> Emmerich           30 km
%   7  Emmerich          -> Wesel              35 km
%   8  Wesel             -> Duisburg           33 km
%
% Total modeled route distance is 235 km.
%
% w_d and U_c are spatial averages of the available kilometre-level
% water-depth and current data within each segment. They represent
% conditions along that segment, not time-varying conditions during
% the voyage. U_c is signed relative to upstream travel: negative
% values indicate an opposing current. The wrapper reverses segment
% order and current sign when downstream travel is selected.

d = [39000, 20000, 42000, 26000, 10000, 30000, 35000, 33000];
                                                    % Segment distance [m]

w_d = [8, 7.90142857142857, 7.70025582790698, 8.22218518518519, ...
       8.09254545454545, 7.99548390322581, 8.35961113055556, ...
       7.83464703235294];                          % Mean water depth [m]

U_c = [-0.19378925, -0.271610952380952, -1.09837581395349, ...
       -0.922359259259259, -0.967033636363636, -1.24912983102919, ...
       -1.46871031746032, -1.57700840336134];   % Mean current [m/s]

N = numel(d);                                      % Number of segments

assert(numel(w_d) == N && numel(U_c) == N, ...
    'd, w_d, and U_c must describe the same number of segments.');
assert(all(isfinite([d, w_d, U_c])) && ...
    all(d > 0) && all(w_d > 0), ...
    'Segment lengths and depths must be positive and all inputs finite.');

%% Water properties and gravity
% Change rho and vis if the modeled water conditions differ.
g = 9.81;               % Gravitational acceleration [m/s^2]
rho = 1000;             % Water density [kg/m^3]
vis = 0.0000012696;     % Water kinematic viscosity [m^2/s]

assert(all(isfinite([g, rho, vis])) && all([g, rho, vis] > 0), ...
    'g, rho, and vis must be finite and positive.');