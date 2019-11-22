function [r_t__t_b_K, v_t__t_b_K, a_t__t_b_K, C_t__b_K] = TAN_mech(w_b__i_b_tilde, f_b__i_b_tilde, P_init, V_init, A_init, constants)
%#codegen
% FUNCTION DESCRIPTION:
%   Implements the low and high-fidelity Tangential Frame Mechanization
%
%   From Dr. Bruder's Simulink model
%
% INPUTS:
%   w_b__i_b_tilde  = Current Gyro measurements (rad/s)
%   f_b__i_b_tilde  = Current Accel measurements (m/s^2)
%   P_init          = True position r_t__t_b used to initialize (meters)
%   V_init          = True velocity v_t__t_b used to initialize (meters/sec)
%   A_init          = True attitude C_t__b used to initialize (dimless)
%
% OUTPUTS:
%   r_t__t_b_K      = Updated position r_t__t_b(k) = r_t__t_b(+) (meters)
%   v_t__t_b_K      = Updated velocity v_t__t_b(k) = v_t__t_b(+) (meters/sec)
%   C_t__b_K        = Updated attitude C_t__b(k)   = C_t__b(+)
%
%  Reference:  EE 440 Lecture notes:
%
%  NOTES:
%   1.) fidelity = 0 => Low-fidelity or = 1 => High-fidelity
%   2.) The True PVA is ONLY used to initialize the mechanization!!

persistent r_t__t_b_K_1 v_t__t_b_K_1 C_t__b_K_1

fidelity = constants.fidelity;      % Set to '0' for Low-fidelity and '1' for High-fidelity
dt       = constants.dt;            % Time step (sec)
I3       = eye(3);                  % 3X3 Identity matrix
Ohm_i__i_e = constants.Ohm_i__i_e;  % Earth Rate skew-symmetric matrix (rad/s)
Ohm_e__i_e = Ohm_i__i_e;
w_i__i_e   = constants.w_i__i_e;    % Angular velocity of {e} wrt {i} resolved in {i} (rad/s)
w_e__i_e = w_i__i_e;
C_e__t = constants.C_e__t;          % Attitude of t-frame wrt e-frame (a constant matrix)
C_t__e = C_e__t';
r_e__e_t = constants.r_e__e_t;      % Position of org of t-frame wrt e-frame resolved in e-frame

% If this is the first call to the function then Initialize the PVA
if isempty(C_t__b_K_1)      % Initialize the PVA (with ground truth)
    r_t__t_b_K = P_init;
    v_t__t_b_K = V_init;
    a_t__t_b_K = f_b__i_b_tilde;
    C_t__b_K   = A_init;
    
    % Update PVA(k-1) = PVA(-) values
    r_t__t_b_K_1 = P_init; % Body and nav frame share common org
    v_t__t_b_K_1 = V_init;
    C_t__b_K_1   = A_init;
else
%==========================================================================
% Tangential Frame PVA Mechanization:
%==========================================================================
    Ohm_t__i_e = C_t__e * Ohm_i__i_e * C_e__t; % Earth rate in the t-frame
    Ohm_b__i_b = vec2ss(w_b__i_b_tilde);
    
    %----------------------------------------------------------------------
    % STEP 1.) Attitude Update
    %----------------------------------------------------------------------    
    if ~fidelity                    % If "low-fidelity" eqns

        % First order approximation used (Lower fidelity eqns) ++++++++++++++++++++

        C_t__b_K = C_t__b_K_1*(I3 + Ohm_b__i_b*dt) - (C_t__e*Ohm_e__i_e*C_e__t)*C_t__b_K_1*dt;

    else                            % If "high-fidelity" eqns
        
        % Higher fidelity equations +++++++++++++++++++++++++++++++++++++++++++++++

        w_t__t_b = C_t__b_K_1*w_b__i_b_tilde - C_t__e*w_e__i_e;
        k_hat = w_t__t_b / norm(w_t__t_b);
        K = vec2ss(k_hat);
        d_theta = norm(w_t__t_b) * dt;
        
        C_t__b_K =(I3 + sin(d_theta)*K + (1-cos(d_theta))*K^2)*C_t__b_K_1;
        
    end
    
    %--------------------------------------------------------------------------
    % STEP 2.) Specific Force Update
    %----------------------------------------------------------------------

    f_t__i_b = C_t__b_K  * f_b__i_b_tilde;   % (m/s^2)

    %--------------------------------------------------------------------------
    % STEP 3.) Velocity Update
    %----------------------------------------------------------------------
    % Gravity Calculation:
    % Since the body is above the ellipsoid use the "gravity" fn NOT Somigliana
    r_e__e_b = r_e__e_t + C_e__t * r_t__t_b_K_1;            % Compute r_e__e_b
    [L_b, lambda_b, h_b] = xyz2llh(r_e__e_b, constants);    % Compute the lat, lon, and height
%     g_n__b = [0; 0; gravity(L_b, h_b, constants)];          % Compute the acceleration due to gravity
    g_n__b = [0; 0; constants.gravity];          % Compute the acceleration due to gravity
    C_e__n = Lat_Lon_2C_e__n(L_b, lambda_b);                % Compute C_e__n
    g_e__b = C_e__n * g_n__b;                               % Compute the gravity of the body in the {e} frame
    g_t__b = C_t__e * g_e__b;                               % Compute the gravity of the body in the {t} frame
    

    a_t__t_b = f_t__i_b + g_t__b - 2*C_t__e*Ohm_i__i_e*C_e__t*v_t__t_b_K_1; % (m/s^2)
    a_t__t_b_K = a_t__t_b;

    v_t__t_b_K = v_t__t_b_K_1 + a_t__t_b*dt;  % (m/s)

    %--------------------------------------------------------------------------
    % STEP 4.) Position Update
    %----------------------------------------------------------------------
    r_t__t_b_K = r_t__t_b_K_1 + v_t__t_b_K_1*dt + a_t__t_b*dt^2/2; % slide 8 (m)
    
    % Store PVA(k-1) values for use in next iteration
    r_t__t_b_K_1 = r_t__t_b_K; % Body and nav frame share common org
    v_t__t_b_K_1 = v_t__t_b_K;
    C_t__b_K_1   = C_t__b_K;
end  % End of the t_sec == 0 block

end     % End of function