function out = function_squat(x, z, V_ow, C_B, T, B, w_d)
    
    % Squat calculations as presented in (Serban et al., 2015) "The analysis
    % of squat and underkeel clearance for different ship types in a trapezoidal crosssection channel"
    W_0 = 100;                                                              % Channel width at surface
    B_c = 50;                                                               % Channel width at bottom
    A_c = ((W_0 + B_c)*w_d)/2;                                              % Cross sectional area of a channel (trapezoidal)
    A_s = T*B;                                                              % Cross sectional area of the immersed part of the ship
    S_1 = A_s/A_c;                                                          % Blockage factor
    K = 6*S_1 + 0.40;                                                       % Blockage parameter
    h_1 = 0;      %(C_B * S_1^0.81 * V_ow{z}(x)^2.08)/20;                   % Alternative squat calculation 1                             
    h_2 = 0;      %(C_B * V_ow{z}(x)^2)/50;                                 % Alternative squat calculation 2
    h_3 = K*(C_B * V_ow{z}(x)^2)/100;                                                        

    out = max([h_1,h_2,h_3]);

end