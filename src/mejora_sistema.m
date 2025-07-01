function [k_mejorado, c_mejorado, resultados_mejora] = mejora_sistema(parametros, k_inicial, c_inicial)
% MEJORA_SISTEMA    Estrategias de optimización y selección de la mejor
%
%   [k_mejorado, c_mejorado, resultados_mejora] = ...
%       mejora_sistema(parametros, k_inicial, c_inicial)
%
% Entradas:
%   parametros   - Estructura con campos m, v0, f_max, carrera_max, t_amort_max, F_ext
%   k_inicial    - Valor inicial de rigidez [N/m]
%   c_inicial    - Valor inicial de amortiguamiento [N·s/m]
%
% Salidas:
%   k_mejorado         - Rigidez del amortiguador de la mejor estrategia [N/m]
%   c_mejorado         - Amortiguamiento de la mejor estrategia [N·s/m]
%   resultados_mejora  - Estructura con métricas y función objetivo de la estrategia elegida

    % 1) Evaluar desempeño de la configuración inicial
    resultados_inicial = evaluar_desempeno(parametros, k_inicial, c_inicial);

    % 2) Estrategia clásica
    [k_clas, c_clas, resultados_clas] = optimizacion_clasica(parametros, k_inicial, c_inicial);

    % 3) Estrategia adaptativa
    [k_adap, c_adap, resultados_adap] = amortiguacion_adaptativa(parametros, k_inicial, c_inicial);

    % 4) Estrategia doble cámara
    [k_dob, c_dob, resultados_dob] = sistema_doble_camara(parametros, k_inicial, c_inicial);

    % 5) Estrategia semi-activa
    [k_semi, c_semi, resultados_semi] = control_semi_activo(parametros, k_inicial, c_inicial);

    % Agrupar opciones
    ks = {k_inicial, k_clas,    k_adap,    k_dob,    k_semi};
    cs = {c_inicial, c_clas,    c_adap,    c_dob,    c_semi};
    resultados = { resultados_inicial, ...
                   resultados_clas, ...
                   resultados_adap, ...
                   resultados_dob, ...
                   resultados_semi };

    % 6) Seleccionar la mejor estrategia factible
    [~, parametros_opt, resultados_mejora] = seleccionar_mejor_estrategia( ...
        parametros, ks, cs, resultados );

    % Extraer k y c de la estrategia ganadora
    k_mejorado = parametros_opt.k;
    c_mejorado = parametros_opt.c;
    resultados_mejora.F_max_mejorado     = resultados_mejora.F_max;
resultados_mejora.carrera_max_mejorado = resultados_mejora.carrera_max;

end

function resultados = evaluar_desempeno(parametros, k, c)
% EVALUAR_DESEMPENO Evaluación completa del desempeño del sistema
    
    % Simulación del sistema
    [t, u, u_dot, F_amort] = simular_sistema_optimizacion(parametros, k, c);
    
    % Métricas de desempeño
    resultados.k = k;
    resultados.c = c;
    resultados.F_max = max(abs(F_amort));
    resultados.carrera_max = max(abs(u));
    resultados.t_establecimiento = calcular_tiempo_establecimiento(t, u);
    resultados.sobrepaso = calcular_sobrepaso(u);
    resultados.energia_disipada = calcular_energia_total_disipada(t, u_dot, c);
    
    % Función objetivo (menor es mejor)
    resultados.funcion_objetivo = calcular_funcion_objetivo(resultados, parametros);
    
    % Verificación de restricciones
    resultados.cumple_fuerza = resultados.F_max <= parametros.f_max;
    resultados.cumple_carrera = resultados.carrera_max <= parametros.carrera_max;
    resultados.cumple_tiempo = resultados.t_establecimiento <= parametros.t_amort_max;
    resultados.factible = resultados.cumple_fuerza && resultados.cumple_carrera && resultados.cumple_tiempo;

end

function [t, u, u_dot, F_amort] = simular_sistema_optimizacion(parametros, k, c)
% SIMULAR_SISTEMA_OPTIMIZACION Simulación rápida para optimización
    
    % Parámetros de simulación
    t_final = 5.0;
    dt = 0.005;
    t = 0:dt:t_final;
    n = length(t);
    
    % Inicialización
    u = zeros(n, 1);
    u_dot = zeros(n, 1);
    u(1) = 0;
    u_dot(1) = parametros.v0;
    
    % Integración por método de Runge-Kutta de 4to orden
    for i = 1:(n-1)
        [u(i+1), u_dot(i+1)] = runge_kutta_4_paso(u(i), u_dot(i), dt, parametros, k, c);
    end
    
    % Fuerza del amortiguador
    F_amort = c * abs(u_dot) + k * abs(u);

end

function [u_new, u_dot_new] = runge_kutta_4_paso(u, u_dot, dt, parametros, k, c)
% RUNGE_KUTTA_4_PASO Un paso del método RK4 para el sistema masa-resorte-amortiguador
    
    % Función de la ecuación diferencial: u_ddot = f(u, u_dot)
    f = @(x, v) (parametros.F_ext - c*v - k*x) / parametros.m;
    
    % Coeficientes RK4
    k1_v = f(u, u_dot);
    k1_x = u_dot;
    
    k2_v = f(u + 0.5*dt*k1_x, u_dot + 0.5*dt*k1_v);
    k2_x = u_dot + 0.5*dt*k1_v;
    
    k3_v = f(u + 0.5*dt*k2_x, u_dot + 0.5*dt*k2_v);
    k3_x = u_dot + 0.5*dt*k2_v;
    
    k4_v = f(u + dt*k3_x, u_dot + dt*k3_v);
    k4_x = u_dot + dt*k3_v;
    
    % Actualización
    u_new = u + (dt/6) * (k1_x + 2*k2_x + 2*k3_x + k4_x);
    u_dot_new = u_dot + (dt/6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);

end

function [k_opt, c_opt, resultados] = optimizacion_clasica(parametros, k0, c0)
% OPTIMIZACION_CLASICA Optimización mediante algoritmo de búsqueda directa
    
    fprintf('Ejecutando optimización clásica...\n');
    
    % Definir rangos de búsqueda
    k_range = linspace(k0*0.5, k0*2.0, 25);
    c_range = linspace(c0*0.5, c0*2.0, 25);
    
    mejor_funcion_objetivo = inf;
    k_opt = k0;
    c_opt = c0;
    
    % Búsqueda exhaustiva en grilla
    for i = 1:length(k_range)
        for j = 1:length(c_range)
            resultado_temp = evaluar_desempeno(parametros, k_range(i), c_range(j));
            
            % Solo considerar soluciones factibles
            if resultado_temp.factible && resultado_temp.funcion_objetivo < mejor_funcion_objetivo
                mejor_funcion_objetivo = resultado_temp.funcion_objetivo;
                k_opt = k_range(i);
                c_opt = c_range(j);
                resultados = resultado_temp;
            end
        end
    end
    
    % Si no se encontró solución factible, usar la menos mala
    if mejor_funcion_objetivo == inf
        fprintf('⚠️  No se encontró solución factible. Usando mejor compromiso.\n');
        mejor_penalizacion = inf;
        for i = 1:length(k_range)
            for j = 1:length(c_range)
                resultado_temp = evaluar_desempeno(parametros, k_range(i), c_range(j));
                penalizacion = calcular_penalizacion(resultado_temp, parametros);
                
                if penalizacion < mejor_penalizacion
                    mejor_penalizacion = penalizacion;
                    k_opt = k_range(i);
                    c_opt = c_range(j);
                    resultados = resultado_temp;
                end
            end
        end
    end
    
    fprintf('Optimización clásica completada.\n');
    fprintf('k óptimo: %.0f N/m, c óptimo: %.0f N⋅s/m\n', k_opt, c_opt);

end

function [k_opt, c_opt, resultados] = amortiguacion_adaptativa(parametros, k0, c0)
% AMORTIGUACION_ADAPTATIVA Sistema con amortiguación variable según velocidad
    
    fprintf('Diseñando sistema de amortiguación adaptativa...\n');
    
    % Concepto: c = c_base + c_variable * |v|^n
    % Esto simula un orificio variable o válvula adaptativa
    
    c_base_range = linspace(c0*0.3, c0*0.8, 15);
    c_var_range = linspace(c0*0.1, c0*0.7, 15);
    k_range = linspace(k0*0.8, k0*1.5, 10);
    exponente_range = [0.5, 1.0, 1.5, 2.0];
    
    mejor_resultado = inf;
    k_opt = k0;
    c_opt = c0;
    
    for i = 1:length(k_range)
        for j = 1:length(c_base_range)
            for l = 1:length(c_var_range)
                for m = 1:length(exponente_range)
                    % Simular con amortiguación adaptativa
                    resultado_temp = simular_amortiguacion_adaptativa(parametros, k_range(i), ...
                        c_base_range(j), c_var_range(l), exponente_range(m));
                    
                    if resultado_temp.factible && resultado_temp.funcion_objetivo < mejor_resultado
                        mejor_resultado = resultado_temp.funcion_objetivo;
                        k_opt = k_range(i);
                        c_opt = c_base_range(j) + c_var_range(l); % Equivalente promedio
                        resultados = resultado_temp;
                        resultados.c_base = c_base_range(j);
                        resultados.c_variable = c_var_range(l);
                        resultados.exponente = exponente_range(m);
                    end
                end
            end
        end
    end
    
    if mejor_resultado == inf
        fprintf('⚠️  Sistema adaptativo no mejoró el desempeño.\n');
        resultados = evaluar_desempeno(parametros, k0, c0);
        k_opt = k0;
        c_opt = c0;
    else
        fprintf('Sistema adaptativo optimizado.\n');
        fprintf('c_base: %.0f N⋅s/m, c_variable: %.0f N⋅s/m\n', ...
                resultados.c_base, resultados.c_variable);
    end

end

function resultado = simular_amortiguacion_adaptativa(parametros, k, c_base, c_var, exponente)
% SIMULAR_AMORTIGUACION_ADAPTATIVA Simulación con coeficiente de amortiguación variable
    
    % Simulación modificada
    t_final = 5.0;
    dt = 0.005;
    t = 0:dt:t_final;
    n = length(t);
    
    u = zeros(n, 1);
    u_dot = zeros(n, 1);
    u(1) = 0;
    u_dot(1) = parametros.v0;
    
    F_amort = zeros(n, 1);
    
    for i = 1:(n-1)
        % Amortiguación adaptativa
        c_efectivo = c_base + c_var * abs(u_dot(i))^exponente;
        
        % Ecuación de movimiento
        u_ddot = (parametros.F_ext - c_efectivo*u_dot(i) - k*u(i)) / parametros.m;
        
        % Integración
        u_dot(i+1) = u_dot(i) + u_ddot * dt;
        u(i+1) = u(i) + u_dot(i) * dt;
        
        % Fuerza del amortiguador
        F_amort(i) = c_efectivo * abs(u_dot(i)) + k * abs(u(i));
    end
    
    % Última fuerza
    c_efectivo = c_base + c_var * abs(u_dot(end))^exponente;
    F_amort(end) = c_efectivo * abs(u_dot(end)) + k * abs(u(end));
    
    % Evaluar métricas
    resultado.k = k;
    resultado.c = c_base + c_var; % Equivalente promedio
    resultado.F_max = max(abs(F_amort));
    resultado.carrera_max = max(abs(u));
    resultado.t_establecimiento = calcular_tiempo_establecimiento(t, u);
    resultado.sobrepaso = calcular_sobrepaso(u);
    resultado.energia_disipada = calcular_energia_total_disipada(t, u_dot, c_base + c_var);
    
    % Función objetivo
    resultado.funcion_objetivo = calcular_funcion_objetivo(resultado, parametros);
    
    % Verificación de restricciones
    resultado.cumple_fuerza = resultado.F_max <= parametros.f_max;
    resultado.cumple_carrera = resultado.carrera_max <= parametros.carrera_max;
    resultado.cumple_tiempo = resultado.t_establecimiento <= parametros.t_amort_max;
    resultado.factible = resultado.cumple_fuerza && resultado.cumple_carrera && resultado.cumple_tiempo;

end

function [k_opt, c_opt, resultados] = sistema_doble_camara(parametros, k0, c0)
% SISTEMA_DOBLE_CAMARA Sistema con dos cámaras de amortiguación en paralelo
    
    fprintf('Diseñando sistema de doble cámara...\n');
    
    % Concepto: Dos amortiguadores en paralelo con diferentes características
    % c_total = c1 + c2, pero con activación secuencial
    
    % Parámetros de búsqueda
    k_range = linspace(k0*0.7, k0*1.3, 12);
    c1_range = linspace(c0*0.2, c0*0.6, 12);
    c2_range = linspace(c0*0.3, c0*0.8, 12);
    umbral_range = linspace(0.1, 0.5, 8); % Umbral de activación de segunda cámara
    
    mejor_resultado = inf;
    k_opt = k0;
    c_opt = c0;
    
    for i = 1:length(k_range)
        for j = 1:length(c1_range)
            for l = 1:length(c2_range)
                for m = 1:length(umbral_range)
                    resultado_temp = simular_sistema_doble_camara(parametros, k_range(i), ...
                        c1_range(j), c2_range(l), umbral_range(m));
                    
                    if resultado_temp.factible && resultado_temp.funcion_objetivo < mejor_resultado
                        mejor_resultado = resultado_temp.funcion_objetivo;
                        k_opt = k_range(i);
                        c_opt = c1_range(j) + c2_range(l);
                        resultados = resultado_temp;
                        resultados.c1 = c1_range(j);
                        resultados.c2 = c2_range(l);
                        resultados.umbral_activacion = umbral_range(m);
                    end
                end
            end
        end
    end
    
    if mejor_resultado == inf
        fprintf('⚠️  Sistema de doble cámara no mejoró el desempeño.\n');
        resultados = evaluar_desempeno(parametros, k0, c0);
        k_opt = k0;
        c_opt = c0;
    else
        fprintf('Sistema de doble cámara optimizado.\n');
        fprintf('c1: %.0f N⋅s/m, c2: %.0f N⋅s/m\n', resultados.c1, resultados.c2);
    end

end

function resultado = simular_sistema_doble_camara(parametros, k, c1, c2, umbral)
% SIMULAR_SISTEMA_DOBLE_CAMARA Simulación con sistema de doble cámara
    
    t_final = 5.0;
    dt = 0.005;
    t = 0:dt:t_final;
    n = length(t);
    
    u = zeros(n, 1);
    u_dot = zeros(n, 1);
    u(1) = 0;
    u_dot(1) = parametros.v0;
    
    F_amort = zeros(n, 1);
    
    for i = 1:(n-1)
        % Activación de segunda cámara basada en velocidad
        if abs(u_dot(i)) > umbral
            c_efectivo = c1 + c2; % Ambas cámaras activas
        else
            c_efectivo = c1; % Solo primera cámara
        end
        
        % Ecuación de movimiento
        u_ddot = (parametros.F_ext - c_efectivo*u_dot(i) - k*u(i)) / parametros.m;
        
        % Integración
        u_dot(i+1) = u_dot(i) + u_ddot * dt;
        u(i+1) = u(i) + u_dot(i) * dt;
        
        % Fuerza del amortiguador
        F_amort(i) = c_efectivo * abs(u_dot(i)) + k * abs(u(i));
    end
    
    % Última fuerza
    if abs(u_dot(end)) > umbral
        c_efectivo = c1 + c2;
    else
        c_efectivo = c1;
    end
    F_amort(end) = c_efectivo * abs(u_dot(end)) + k * abs(u(end));
    
    % Evaluar métricas
    resultado.k = k;
    resultado.c = c1 + c2;
    resultado.F_max = max(abs(F_amort));
    resultado.carrera_max = max(abs(u));
    resultado.t_establecimiento = calcular_tiempo_establecimiento(t, u);
    resultado.sobrepaso = calcular_sobrepaso(u);
    resultado.energia_disipada = calcular_energia_total_disipada(t, u_dot, c1 + c2);
    
    % Función objetivo
    resultado.funcion_objetivo = calcular_funcion_objetivo(resultado, parametros);
    
    % Verificación de restricciones
    resultado.cumple_fuerza = resultado.F_max <= parametros.f_max;
    resultado.cumple_carrera = resultado.carrera_max <= parametros.carrera_max;
    resultado.cumple_tiempo = resultado.t_establecimiento <= parametros.t_amort_max;
    resultado.factible = resultado.cumple_fuerza && resultado.cumple_carrera && resultado.cumple_tiempo;

end

function [k_opt, c_opt, resultados] = control_semi_activo(parametros, k0, c0)
% CONTROL_SEMI_ACTIVO Sistema con control semi-activo simulado
    
    fprintf('Diseñando sistema de control semi-activo...\n');
    
    % Concepto: Amortiguación variable basada en algoritmo de control
    % c = c_min + (c_max - c_min) * factor_control
    
    k_range = linspace(k0*0.8, k0*1.2, 10);
    c_min_range = linspace(c0*0.2, c0*0.5, 10);
    c_max_range = linspace(c0*1.2, c0*2.0, 10);
    
    mejor_resultado = inf;
    k_opt = k0;
    c_opt = c0;
    
    for i = 1:length(k_range)
        for j = 1:length(c_min_range)
            for l = 1:length(c_max_range)
                if c_max_range(l) > c_min_range(j) % Validación lógica
                    resultado_temp = simular_control_semi_activo(parametros, k_range(i), ...
                        c_min_range(j), c_max_range(l));
                    
                    if resultado_temp.factible && resultado_temp.funcion_objetivo < mejor_resultado
                        mejor_resultado = resultado_temp.funcion_objetivo;
                        k_opt = k_range(i);
                        c_opt = (c_min_range(j) + c_max_range(l)) / 2; % Promedio
                        resultados = resultado_temp;
                        resultados.c_min = c_min_range(j);
                        resultados.c_max = c_max_range(l);
                    end
                end
            end
        end
    end
    
    if mejor_resultado == inf
        fprintf('⚠️  Control semi-activo no mejoró el desempeño.\n');
        resultados = evaluar_desempeno(parametros, k0, c0);
        k_opt = k0;
        c_opt = c0;
    else
        fprintf('Control semi-activo optimizado.\n');
        fprintf('c_min: %.0f N⋅s/m, c_max: %.0f N⋅s/m\n', resultados.c_min, resultados.c_max);
    end

end

function resultado = simular_control_semi_activo(parametros, k, c_min, c_max)
% SIMULAR_CONTROL_SEMI_ACTIVO Simulación con control semi-activo
    
    t_final = 5.0;
    dt = 0.005;
    t = 0:dt:t_final;
    n = length(t);
    
    u = zeros(n, 1);
    u_dot = zeros(n, 1);
    u(1) = 0;
    u_dot(1) = parametros.v0;
    
    F_amort = zeros(n, 1);
    
    for i = 1:(n-1)
        % Algoritmo de control: Skyhook damping simplificado
        % Aumentar amortiguación cuando velocidad y aceleración tienen mismo signo
        u_ddot_prev = (parametros.F_ext - c_min*u_dot(i) - k*u(i)) / parametros.m;
        
        if u_dot(i) * u_ddot_prev > 0
            % Misma dirección: usar amortiguación máxima
            factor_control = 1.0;
        else
            % Direcciones opuestas: usar amortiguación mínima
            factor_control = 0.3;
        end
        
        c_efectivo = c_min + (c_max - c_min) * factor_control;
        
        % Ecuación de movimiento
        u_ddot = (parametros.F_ext - c_efectivo*u_dot(i) - k*u(i)) / parametros.m;
        
        % Integración
        u_dot(i+1) = u_dot(i) + u_ddot * dt;
        u(i+1) = u(i) + u_dot(i) * dt;
        
        % Fuerza del amortiguador
        F_amort(i) = c_efectivo * abs(u_dot(i)) + k * abs(u(i));
    end
    
    % Última fuerza
    F_amort(end) = c_min * abs(u_dot(end)) + k * abs(u(end));
    
    % Evaluar métricas
    resultado.k = k;
    resultado.c = (c_min + c_max) / 2;
    resultado.F_max = max(abs(F_amort));
    resultado.carrera_max = max(abs(u));
    resultado.t_establecimiento = calcular_tiempo_establecimiento(t, u);
    resultado.sobrepaso = calcular_sobrepaso(u);
    resultado.energia_disipada = calcular_energia_total_disipada(t, u_dot, resultado.c);
    
    % Función objetivo
    resultado.funcion_objetivo = calcular_funcion_objetivo(resultado, parametros);
    
    % Verificación de restricciones
    resultado.cumple_fuerza = resultado.F_max <= parametros.f_max;
    resultado.cumple_carrera = resultado.carrera_max <= parametros.carrera_max;
    resultado.cumple_tiempo = resultado.t_establecimiento <= parametros.t_amort_max;
    resultado.factible = resultado.cumple_fuerza && resultado.cumple_carrera && resultado.cumple_tiempo;

end

function [mejor_estrategia, parametros_opt, resultados_opt] = seleccionar_mejor_estrategia(parametros, k_array, c_array, resultados_array)
% SELECCIONAR_MEJOR_ESTRATEGIA Compara todas las estrategias y selecciona la mejor
    
    fprintf('\n--- COMPARACIÓN DE ESTRATEGIAS ---\n');
    
    estrategias = {'Inicial', 'Clásica', 'Adaptativa', 'Doble Cámara', 'Semi-Activo'};
    
    mejor_funcion_objetivo = inf;
    mejor_idx = 1;
    
    for i = 1:length(estrategias)
        fprintf('%s: ', estrategias{i});
        
        if resultados_array{i}.factible
            fprintf('Factible - F.O. = %.3f\n', resultados_array{i}.funcion_objetivo);
            if resultados_array{i}.funcion_objetivo < mejor_funcion_objetivo
                mejor_funcion_objetivo = resultados_array{i}.funcion_objetivo;
                mejor_idx = i;
            end
        else
            fprintf('No factible\n');
        end
    end
    
    mejor_estrategia = estrategias{mejor_idx};
    parametros_opt.k = k_array{mejor_idx};
    parametros_opt.c = c_array{mejor_idx};
    resultados_opt = resultados_array{mejor_idx};
    
    fprintf('\n🏆 MEJOR ESTRATEGIA: %s\n', mejor_estrategia);

end

function mostrar_resultados_evaluacion(titulo, resultados)
% MOSTRAR_RESULTADOS_EVALUACION Muestra resultados de evaluación formateados
    
    fprintf('\n--- %s ---\n', titulo);
    fprintf('Fuerza máxima:          %.1f N (%s)\n', resultados.F_max, ...
            evaluar_cumplimiento_restriccion(resultados.cumple_fuerza));
    fprintf('Carrera máxima:         %.1f mm (%s)\n', resultados.carrera_max*1000, ...
            evaluar_cumplimiento_restriccion(resultados.cumple_carrera));
    fprintf('Tiempo establecimiento: %.2f s (%s)\n', resultados.t_establecimiento, ...
            evaluar_cumplimiento_restriccion(resultados.cumple_tiempo));
    fprintf('Sobrepaso:              %.1f%\n', resultados.sobrepaso*100);
    fprintf('Energía disipada:       %.1f J\n', resultados.energia_disipada);
    fprintf('Función objetivo:       %.3f\n', resultados.funcion_objetivo);
    fprintf('Factibilidad:           %s\n', evaluar_cumplimiento_restriccion(resultados.factible));

end

function estado = evaluar_cumplimiento_restriccion(cumple)
% EVALUAR_CUMPLIMIENTO_RESTRICCION Evalúa si se cumple una restricción
    if cumple
        estado = '✓';
    else
        estado = '✗';
    end
end

function generar_reporte_final(estrategia, parametros_opt, resultados)
% GENERAR_REPORTE_FINAL Genera el reporte final de optimización
    
    fprintf('\n=== REPORTE FINAL DE OPTIMIZACIÓN ===\n');
    fprintf('Estrategia seleccionada: %s\n', estrategia);
    fprintf('\nParámetros optimizados:\n');
    fprintf('- Rigidez (k):           %.0f N/m\n', parametros_opt.k);
    fprintf('- Amortiguación (c):     %.0f N⋅s/m\n', parametros_opt.c);
    
    fprintf('\nDesempeño del sistema optimizado:\n');
    mostrar_resultados_evaluacion('SISTEMA OPTIMIZADO', resultados);
    
    % Recomendaciones adicionales
    fprintf('\n--- RECOMENDACIONES TÉCNICAS ---\n');
    generar_recomendaciones_tecnicas(estrategia, resultados);

end

function generar_recomendaciones_tecnicas(estrategia, resultados)
% GENERAR_RECOMENDACIONES_TECNICAS Genera recomendaciones específicas
    
    switch estrategia
        case 'Clásica'
            fprintf('• Implementar sistema convencional con parámetros optimizados\n');
            fprintf('• Considerar tolerancias de manufactura ±5%\n');
            
        case 'Adaptativa'
            fprintf('• Implementar válvula de orificio variable\n');
            fprintf('• Requiere sistema de control de flujo\n');
            
        case 'Doble Cámara'
            fprintf('• Diseñar sistema de dos cámaras en paralelo\n');
            fprintf('• Implementar válvula de activación por velocidad\n');
            
        case 'Semi-Activo'
            fprintf('• Requiere sistema de control electrónico\n');
            fprintf('• Implementar sensores de velocidad y aceleración\n');
            
        otherwise
            fprintf('• Mantener configuración inicial\n');
            fprintf('• Considerar optimización manual de parámetros\n');
    end
    
    % Recomendaciones generales
    fprintf('\n--- RECOMENDACIONES GENERALES ---\n');
    fprintf('• Realizar pruebas experimentales para validación\n');
    fprintf('• Considerar efectos de temperatura en fluido hidráulico\n');
    fprintf('• Implementar sistema de monitoreo de desempeño\n');
    fprintf('• Planificar mantenimiento preventivo cada 500 horas\n');

end

function generar_graficos_comparativos(parametros_inicial, parametros_opt, resultados)
% GENERAR_GRAFICOS_COMPARATIVOS Genera gráficos de comparación
    
    % Simular ambos sistemas para comparación
    [t_inicial, u_inicial, u_dot_inicial, F_inicial] = ...
        simular_sistema_optimizacion(parametros_inicial, parametros_inicial.k, parametros_inicial.c);
    
    [t_opt, u_opt, u_dot_opt, F_opt] = ...
        simular_sistema_optimizacion(parametros_inicial, parametros_opt.k, parametros_opt.c);
    
    % Crear figura comparativa
    figure('Name', 'Comparación: Sistema Inicial vs Optimizado', 'NumberTitle', 'off');
    set(gcf, 'Position', [100, 100, 1400, 900]);
    
    % Gráfico 1: Desplazamiento
    subplot(2, 3, 1);
    plot(t_inicial, u_inicial*1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Inicial');
    hold on; grid on;
    plot(t_opt, u_opt*1000, 'b-', 'LineWidth', 2, 'DisplayName', 'Optimizado');
    xlabel('Tiempo [s]');
    ylabel('Desplazamiento [mm]');
    title('Comparación de Desplazamiento');
    legend('Location', 'best');
    
    % Gráfico 2: Velocidad
    subplot(2, 3, 2);
    plot(t_inicial, u_dot_inicial, 'r--', 'LineWidth', 2, 'DisplayName', 'Inicial');
    hold on; grid on;
    plot(t_opt, u_dot_opt, 'b-', 'LineWidth', 2, 'DisplayName', 'Optimizado');
    xlabel('Tiempo [s]');
    ylabel('Velocidad [m/s]');
    title('Comparación de Velocidad');
    legend('Location', 'best');
    
    % Gráfico 3: Fuerza del amortiguador
    subplot(2, 3, 3);
    plot(t_inicial, F_inicial/1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Inicial');
    hold on; grid on;
    plot(t_opt, F_opt/1000, 'b-', 'LineWidth', 2, 'DisplayName', 'Optimizado');
    plot([0, max(t_opt)], [parametros_inicial.f_max/1000, parametros_inicial.f_max/1000], ...
         'k:', 'LineWidth', 1, 'DisplayName', 'Límite');
    xlabel('Tiempo [s]');
    ylabel('Fuerza [kN]');
    title('Comparación de Fuerza');
    legend('Location', 'best');
    
    % Gráfico 4: Diagrama de fases
    subplot(2, 3, 4);
    plot(u_inicial*1000, u_dot_inicial, 'r--', 'LineWidth', 2, 'DisplayName', 'Inicial');
    hold on; grid on;
    plot(u_opt*1000, u_dot_opt, 'b-', 'LineWidth', 2, 'DisplayName', 'Optimizado');
    xlabel('Desplazamiento [mm]');
    ylabel('Velocidad [m/s]');
    title('Diagrama de Fases');
    legend('Location', 'best');
    
    % Gráfico 5: Comparación de métricas
    subplot(2, 3, [5, 6]);
    metricas = {'F_{max} [kN]', 'x_{max} [mm]', 't_{est} [s]', 'Sobrepaso [%]'};
    
    % Valores para gráfico de barras
    valores_inicial = [max(F_inicial)/1000, max(abs(u_inicial))*1000, ...
                      calcular_tiempo_establecimiento(t_inicial, u_inicial), ...
                      calcular_sobrepaso(u_inicial)*100];
    
    valores_opt = [max(F_opt)/1000, max(abs(u_opt))*1000, ...
                  calcular_tiempo_establecimiento(t_opt, u_opt), ...
                  calcular_sobrepaso(u_opt)*100];
    
    x = 1:length(metricas);
    width = 0.35;
    
    bar(x - width/2, valores_inicial, width, 'FaceColor', [1, 0.4, 0.4], 'DisplayName', 'Inicial');
    hold on; grid on;
    bar(x + width/2, valores_opt, width, 'FaceColor', [0.4, 0.4, 1], 'DisplayName', 'Optimizado');
    
    set(gca, 'XTick', x);
    set(gca, 'XTickLabel', metricas);
    ylabel('Valor');
    title('Comparación de Métricas de Desempeño');
    legend('Location', 'best');
    
    % Título general
    sgtitle('Análisis Comparativo: Sistema Inicial vs Optimizado', ...
            'FontSize', 14, 'FontWeight', 'bold');

end

% --- Funciones auxiliares ---

function t_est = calcular_tiempo_establecimiento(t, u)
% CALCULAR_TIEMPO_ESTABLECIMIENTO Calcula tiempo de establecimiento (2% del valor final)
    u_final = abs(u(end));
    if u_final < 1e-6
        % Buscar el máximo si el valor final es muy pequeño
        u_max = max(abs(u));
        umbral = 0.02 * u_max;
    else
        umbral = 0.02 * u_final;
    end
    
    idx = find(abs(u) <= umbral, 1, 'last');
    if isempty(idx)
        t_est = t(end);
    else
        t_est = t(idx);
    end
end

function sobrepaso = calcular_sobrepaso(u)
% CALCULAR_SOBREPASO Calcula el sobrepaso del sistema
    u_max = max(abs(u));
    u_final = abs(u(end));
    
    if u_final < 1e-6
        sobrepaso = 0;
    else
        sobrepaso = (u_max - u_final) / u_final;
    end
end

function E_total = calcular_energia_total_disipada(t, u_dot, c)
% CALCULAR_ENERGIA_TOTAL_DISIPADA Calcula energía total disipada
    dt = t(2) - t(1);
    E_total = sum(c * u_dot.^2) * dt;
end

function f_obj = calcular_funcion_objetivo(resultados, parametros)
% CALCULAR_FUNCION_OBJETIVO Calcula función objetivo multicriterio
    
    % Normalización de métricas
    w1 = 0.4; % Peso para tiempo de establecimiento
    w2 = 0.3; % Peso para sobrepaso
    w3 = 0.2; % Peso para fuerza máxima
    w4 = 0.1; % Peso para carrera máxima
    
    % Métricas normalizadas (0 a 1, menor es mejor)
    t_norm = resultados.t_establecimiento / parametros.t_amort_max;
    s_norm = min(resultados.sobrepaso, 1.0); % Limitar sobrepaso
    f_norm = resultados.F_max / parametros.f_max;
    c_norm = resultados.carrera_max / parametros.carrera_max;
    
    % Función objetivo ponderada
    f_obj = w1 * t_norm + w2 * s_norm + w3 * f_norm + w4 * c_norm;
end

function penalizacion = calcular_penalizacion(resultado, parametros)
% CALCULAR_PENALIZACION Calcula penalización para soluciones no factibles
    
    penalizacion = 0;
    
    % Penalización por exceso de fuerza
    if resultado.F_max > parametros.f_max
        penalizacion = penalizacion + (resultado.F_max - parametros.f_max) / parametros.f_max;
    end
    
    % Penalización por exceso de carrera
    if resultado.carrera_max > parametros.carrera_max
        penalizacion = penalizacion + (resultado.carrera_max - parametros.carrera_max) / parametros.carrera_max;
    end
    
    % Penalización por exceso de tiempo
    if resultado.t_establecimiento > parametros.t_amort_max
        penalizacion = penalizacion + (resultado.t_establecimiento - parametros.t_amort_max) / parametros.t_amort_max;
    end
end