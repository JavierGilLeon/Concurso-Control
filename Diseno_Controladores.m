% =========================================================================
% Diseño del control principal
% =========================================================================

clear; clc;

% Parámetros del Modelo 

K   = 5.8;  
tau = 42;   
L   = 10;   

disp('--- PARÁMETROS DE LA PLANTA ---');
fprintf('K = %.2f | tau = %.2f s | L = %.2f s\n\n', K, tau, L);

%% Diseño mediante método AMIGO 

Kp_amigo = (1/K) * ( 0.15 + 0.35*(tau/L) - (tau^2)/(tau+L)^2 );
Ti_amigo = 0.35*L + (13*L*tau^2) / (tau^2 + 12*L*tau + 7*L^2);

disp('--- SINTONÍA AMIGO (PI) ---');
fprintf('Kp = %.4f\n', Kp_amigo);
fprintf('Ti = %.4f s\n\n', Ti_amigo);

%% Diseño mediante método SIMC 

tau_c = L; 
Kp_simc = (1/K) * (tau / (tau_c + L));
Ti_simc = min(tau, 4*(tau_c + L));

disp('--- SINTONÍA SIMC (PI) ---');
fprintf('Kp = %.4f\n', Kp_simc);
fprintf('Ti = %.4f s\n', Ti_simc);


%% 3. Diseño mediante método ITAE ***** MEJOR DE TODOS ****
Kp_itae_pid = (1.357 / K) * (L / tau)^(-0.947);
Ti_itae_pid = (tau / 0.842) * (L / tau)^(0.738);
Td_itae_pid = tau * 0.381 * (L / tau)^(0.995);
Kd_itae_pid = Kp_itae_pid * Td_itae_pid;

disp('------------------------------------------------------');
disp('--- 3. SINTONÍA ITAE (PID con c=0) ---');
fprintf('Kp = %.4f\n', Kp_itae_pid);
fprintf('Ti = %.4f s\n', Ti_itae_pid);
fprintf('Kd = %.4f\n', Kd_itae_pid);


%%
% =========================================================================
% Diseño del feedforward para cada perturbación 
% =========================================================================


clear; clc;

% Ganancias estáticas de las perturbaciones 
K_eva =  1.0000;   % Te_evap
K_tes = -0.2435;   %  Ts_TES
K_tor =  0.3425;   %  Te_torref

% Puntos de Operación de las perturbaciones
op_eva = 20.0;
op_tes = 88.5;
op_tor = 28.0;

% Constantes de compensación
FF_eva = -K_eva / K;
FF_tes = -K_tes / K;
FF_tor = -K_tor / K;

disp('--- CONSTANTES DEL FEEDFORWARD ---');
fprintf('FF_edificio (Te_evap) = %8.4f\n', FF_eva);
fprintf('FF_tanque (Ts_TES)    = %8.4f\n', FF_tes);
fprintf('FF_torre (Te_torref)  = %8.4f\n\n', FF_tor);
