% ==================================================================================================================
% SCRIPT para generar nuestras propias señales 
% ==================================================================================================================

%% NO FUNCIONA ESTE EXPERIMENTO POQUE SATURA 


function senales = GeneradorSenalesNuestras(t)
    
    % Valores base (Punto de Operación)
    Ref_Ts_evap = 13.3282;   % Referencia segura (lejos del límite de 5ºC del bypass)
    Ts_TES      = 85;  % Temp. salida acumulador
    Te_Evap     = 20;  % Temp. entrada evaporador
    Te_Torr_ref = 28;  % Temp. entrada torre refrig.
    T_amb       = 30;  % Temp. ambiente

    % =====================================================================
    % EXPERIMENTO (Simulación de 8000s)
    % =====================================================================
   
    

    if t >= 1000 && t < 3000
        Ref_Ts_evap = 15; 
    end
    
 
    if t >= 3000 && t < 5000
        Ref_Ts_evap = 15; 
        Te_Torr_ref = 31;
    end
    
   
    if t >= 5000 && t < 7000
        Ref_Ts_evap = 15;
        Te_Torr_ref = 31;
        Te_Evap = 28; 
    end
    

    if t >= 7000
        Ref_Ts_evap = 16; 
        Te_Torr_ref = 28;
        Te_Evap = 20;
    end

   
    senales = [Ref_Ts_evap; Ts_TES; Te_Evap; Te_Torr_ref; T_amb];
end