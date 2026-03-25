% =========================================================
% SCRIPT DE SINTONIZACIÓN AUTOMÁTICA PID
% =========================================================
 clc;

% 1. Configuración inicial
nombre_modelo = 'caja_negra'; % ¡Cambia esto por el nombre real sin .slx!

% Valores iniciales para el PID [Kp, Ki, Kd]
% Al ser un sistema de primer orden sin retraso, un PI suele ser suficiente,
% pero dejaremos el PID completo por si acaso.
parametros_iniciales = [1, 1, 0]; 

% Opciones del optimizador (muestra iteraciones en consola)
opciones = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-4, 'MaxIter', 100);

% 2. Ejecutar la optimización
disp('Iniciando la optimización iterativa...');
% fminsearch buscará minimizar lo que devuelve la función 'calcular_costo'
parametros_optimos = fminsearch(@(p) calcular_costo(p, nombre_modelo), parametros_iniciales, opciones);

% 3. Resultados
Kp_opt = parametros_optimos(1);
Ki_opt = parametros_optimos(2);
Kd_opt = parametros_optimos(3);

fprintf('\n¡Optimización completada!\n');
fprintf('Valores ideales encontrados:\n');
fprintf('Kp = %.4f\n', Kp_opt);
fprintf('Ki = %.4f\n', Ki_opt);
fprintf('Kd = %.4f\n', Kd_opt);

% 4. Probar los mejores valores y graficar
assignin('base', 'Kp', Kp_opt);
assignin('base', 'Ki', Ki_opt);
assignin('base', 'Kd', Kd_opt);
simOut = sim(nombre_modelo);
subplot(2,1,1);
plot(simOut.y_data, 'b-', 'LineWidth', 2);
hold on;
plot(simOut.ref, 'r--', 'LineWidth', 2);

subplot(2,1,2);
plot(simOut.u_data, 'b-', 'LineWidth', 2);
title('Respuesta del Sistema con PID Optimizado');
xlabel('Tiempo (s)');
ylabel('Salida (y)');
grid on;

%%
function J = calcular_costo(parametros, nombre_modelo)

assignin('base', 'Kp', parametros(1));
    assignin('base', 'Ki', parametros(2));
    assignin('base', 'Kd', parametros(3));

  if parametros(1) < 0 || parametros(2) < 0 || parametros(3) < 0
        J = 1e6;
        return; 
  end
    
    out = sim(nombre_modelo);

t = out.t_data;
e = out.e_out;
u = out.u_data; 
  
    % 2. RESTRICCIÓN INTELIGENTE: Ignorar el golpe derivativo
        % Supongamos que tu escalón ocurre en t=0. 
        % Vamos a ignorar los primeros 0.05 segundos de la señal de control.
        tiempo_ignorado = 0.2; 
        
        % Filtramos la señal 'u' para quedarnos solo con lo que pasa DESPUÉS del pico
        u_evaluar = u(t > tiempo_ignorado);
        
        % Ahora aplicamos el castigo solo si satura fuera de ese instante inicial
        if ~isempty(u_evaluar) && max(abs(u_evaluar)) >= 9.99
            J = 1e6; 
            return;
        end

    ISU = trapz(t, u.^2); %esfuerzo de la señal de control
    ISE = trapz(t, e.^2);          % Integral del error al cuadrado
    ITAE = trapz(t, t .* abs(e));  % Integral del tiempo por el valor absoluto del error
    
J = 1 * ISE + 1 * ITAE;
      

end