% LABORATORIO INTEGRADOR CON MATLAB - DISEÑO DE AMORTIGUADOR AERONÁUTICO
% =========================================================================
% Proyecto: Diseño de amortiguador para tren de nariz de aeronave
% Modelo: Socata TBM700
% Autores: [Federico, Martin, Liam]
% Fecha: [1-jul-25]
% =========================================================================

% Inicialización del entorno
clear all; clc; close all;
%addpath('src/');

% Banner del proyecto
fprintf('=========================================================================\n');
fprintf('    LABORATORIO INTEGRADOR - DISEÑO DE AMORTIGUADOR AERONÁUTICO\n');
fprintf('                        Socata TBM700\n');
fprintf('=========================================================================\n\n');

% 📋 PARÁMETROS DEL SISTEMA
fprintf('📋 Inicializando parámetros del sistema...\n');
parametros = inicializar_parametros();
mostrar_especificaciones(parametros);

% 📊 FASE I: AJUSTE DE CURVA FUERZA VS VELOCIDAD
fprintf('\n📊 FASE I: Ajuste de curva por mínimos cuadrados\n');
fprintf('------------------------------------------------------\n');
[mejor_ajuste, velocidad, fuerza, error_ajuste] = ajuste_curva();
fprintf('✅ Ajuste de curva completado\n');

% 🔧 FASE II: OPTIMIZACIÓN DE PARÁMETROS K Y C
fprintf('\n🔧 FASE II: Búsqueda de parámetros óptimos\n');
fprintf('------------------------------------------------------\n');
[k_opt, c_opt, error_optimizacion] = optimizacion_kc(velocidad, fuerza, mejor_ajuste, parametros);
fprintf('✅ Optimización de parámetros completada\n');

% 🧮 FASE III: RESOLUCIÓN POR DIFERENCIAS FINITAS
fprintf('\n🧮 FASE III: Simulación del sistema dinámico\n');
fprintf('------------------------------------------------------\n');
[t, u, u_dot, u_ddot, F_amort] = diferencias_finitas(parametros, k_opt, c_opt);
fprintf('✅ Simulación por diferencias finitas completada\n');

% ⚡ FASE IV: ANÁLISIS ENERGÉTICO Y VERIFICACIÓN
fprintf('\n⚡ FASE IV: Análisis energético y verificación\n');
fprintf('------------------------------------------------------\n');
resultados = energia_disipada(t, u, u_dot, F_amort, parametros, k_opt, c_opt);
fprintf('✅ Análisis energético completado\n');

% 🚀 FASE V: PROPUESTA DE MEJORA DEL SISTEMA
fprintf('\n🚀 FASE V: Optimización y mejora del sistema\n');
fprintf('------------------------------------------------------\n');
[k_mejorado, c_mejorado, mejora_resultados] = mejora_sistema(parametros, k_opt, c_opt);
fprintf('✅ Propuesta de mejora completada\n');

% 📈 GENERACIÓN DE GRÁFICOS Y REPORTES
fprintf('\n📈 Generando visualizaciones y reportes...\n');
generar_graficos_completos(t, u, u_dot, F_amort, parametros, resultados, velocidad, fuerza, mejor_ajuste);
generar_reporte_final(resultados, k_opt, c_opt, k_mejorado, c_mejorado, parametros, mejora_resultados);

fprintf('\n=========================================================================\n');
fprintf('                     ANÁLISIS COMPLETADO EXITOSAMENTE\n');
fprintf('=========================================================================\n');

% ===============================
%  FUNCIONES DE INICIALIZACIÓN
% ===============================

function parametros = inicializar_parametros()
    % Especificaciones de la aeronave
    parametros.P_max       = 2900;    % kg - Peso máximo de aterrizaje
    parametros.peso_nariz  = 0.20;    % 20% - Peso sobre tren de nariz
    parametros.v0          = 0.75;    % m/s - Velocidad inicial
    parametros.g           = 9.81;    % m/s² - Aceleración gravitacional
    
    % Condiciones de borde
    parametros.f_max       = 7500;    % N - Fuerza máxima transmitida
    parametros.carrera_max = 0.35;    % m - Carrera máxima del amortiguador
    parametros.t_amort_max = 2.5;     % s - Tiempo máximo de amortiguación
    parametros.precarga    = 2750;    % N - Precarga inicial
    
    % Parámetros calculados
    parametros.m = parametros.P_max * parametros.peso_nariz;  % kg
    parametros.F_ext = -5689.8 + parametros.precarga;        % N
    
    % Parámetros de simulación
    parametros.t_final = 5.0;         % s - Tiempo total de simulación
    parametros.dt      = 0.001;       % s - Paso de tiempo
    parametros.reduccion_objetivo = 0.98;  % 98% de reducción
end

function mostrar_especificaciones(param)
    fprintf('  Especificaciones de la aeronave:\n');
    fprintf('  • Modelo: Socata TBM700\n');
    fprintf('  • Peso máximo aterrizaje: %.0f kg\n', param.P_max);
    fprintf('  • Masa sobre tren nariz: %.1f kg\n', param.m);
    fprintf('  • Velocidad inicial: %.2f m/s\n', param.v0);
    fprintf('\n  Condiciones de borde:\n');
    fprintf('  • Fuerza máxima: %.0f N\n', param.f_max);
    fprintf('  • Carrera máxima: %.0f mm\n', param.carrera_max * 1000);
    fprintf('  • Tiempo máximo: %.1f s\n', param.t_amort_max);
    fprintf('  • Precarga: %.0f N\n', param.precarga);
end

function generar_graficos_completos(t, u, u_dot, F_amort, param, res, vel, fuerza, ajuste)
    % Crear figura principal con subplots
    figure('Position', [100, 100, 1400, 900], 'Name', 'Análisis Completo del Amortiguador');
    
    % 1. Ajuste de curva
    subplot(2, 4, 1);
    plot(vel, fuerza, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    hold on; grid on;
    v_fit = linspace(min(vel), max(vel), 100);
    f_fit = polyval(ajuste, v_fit);
    plot(v_fit, f_fit, 'b-', 'LineWidth', 2);
    xlabel('Velocidad (m/s)'); ylabel('Fuerza (N)');
    title('Ajuste Fuerza vs Velocidad');
    legend({'Datos experimentales', 'Ajuste polinomial'}, 'Location', 'best');
    
    % 2. Desplazamiento vs tiempo
    subplot(2, 4, 2);
    plot(t, u*1000, 'b-', 'LineWidth', 2);
    hold on; plot([0, t(end)], [param.carrera_max*1000, param.carrera_max*1000], 'r--', 'LineWidth', 1.5);
    xlabel('Tiempo (s)'); ylabel('Desplazamiento (mm)');
    title('Evolución del Desplazamiento');
    legend({'Desplazamiento', 'Límite máximo'}, 'Location', 'best');
    grid on;
    
    % 3. Velocidad vs tiempo
    subplot(2, 4, 3);
    plot(t, u_dot, 'g-', 'LineWidth', 2);
    xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
    title('Evolución de la Velocidad');
    grid on;
    
    % 4. Fuerza del amortiguador
    subplot(2, 4, 4);
    plot(t, F_amort/1000, 'r-', 'LineWidth', 2);
    hold on; plot([0, t(end)], [param.f_max/1000, param.f_max/1000], 'k--', 'LineWidth', 1.5);
    xlabel('Tiempo (s)'); ylabel('Fuerza (kN)');
    title('Fuerza del Amortiguador');
    legend({'Fuerza amortiguador', 'Límite máximo'}, 'Location', 'best');
    grid on;
    
    % 5. Evolución de energías
    subplot(2, 4, 5);
    plot(t, res.E_cin, 'b-', 'LineWidth', 2); hold on;
    plot(t, res.E_pot, 'g-', 'LineWidth', 2);
    plot(t, res.E_dis, 'r-', 'LineWidth', 2);
    xlabel('Tiempo (s)'); ylabel('Energía (J)');
    title('Evolución de Energías');
    legend({'Cinética', 'Potencial', 'Disipada'}, 'Location', 'best');
    grid on;
    
    % 6. Diagrama de fases
    subplot(2, 4, 6);
    plot(u*1000, u_dot, 'b-', 'LineWidth', 2);
    xlabel('Desplazamiento (mm)'); ylabel('Velocidad (m/s)');
    title('Diagrama de Fases');
    grid on;
    
    % 7. Balance energético
    subplot(2, 4, 7);
    E_total = res.E_cin + res.E_pot + res.E_dis;
    plot(t, E_total, 'k-', 'LineWidth', 2);
    xlabel('Tiempo (s)'); ylabel('Energía Total (J)');
    title('Conservación de Energía');
    grid on;
    
    % 8. Tasa de disipación
    subplot(2, 4, 8);
    P_disipada = gradient(res.E_dis, t(2)-t(1));
    plot(t, P_disipada, 'm-', 'LineWidth', 2);
    xlabel('Tiempo (s)'); ylabel('Potencia (W)');
    title('Potencia Disipada');
    grid on;
    
    % Ajustar el layout
    sgtitle('Análisis Completo del Sistema de Amortiguación - Socata TBM700', 'FontSize', 14, 'FontWeight', 'bold');
end

function generar_reporte_final(res, k, c, k_mej, c_mej, param, mejora)
    fprintf('\n=========================================================================\n');
    fprintf('                           REPORTE FINAL\n');
    fprintf('=========================================================================\n');
    
    fprintf('\n📊 PARÁMETROS OPTIMIZADOS:\n');
    fprintf('  • Rigidez (k): %.0f N/m\n', k);
    fprintf('  • Amortiguamiento (c): %.0f N⋅s/m\n', c);
    
    fprintf('\n🚀 PARÁMETROS MEJORADOS:\n');
    fprintf('  • Rigidez mejorada (k): %.0f N/m\n', k_mej);
    fprintf('  • Amortiguamiento mejorado (c): %.0f N⋅s/m\n', c_mej);
    
    fprintf('\n✅ EVALUACIÓN DE CUMPLIMIENTO:\n');
    fprintf('  • Fuerza máxima: %.1f N / %.0f N → %s\n', res.F_max, param.f_max, evaluar_cumplimiento(res.cumple_fuerza));
    fprintf('  • Carrera máxima: %.1f mm / %.0f mm → %s\n', res.x_max*1000, param.carrera_max*1000, evaluar_cumplimiento(res.cumple_carrera));
    fprintf('  • Tiempo amortiguación: %.2f s / %.1f s → %s\n', res.t_98, param.t_amort_max, evaluar_cumplimiento(res.cumple_tiempo));
    
    fprintf('\n⚡ ANÁLISIS ENERGÉTICO:\n');
    fprintf('  • Energía inicial: %.2f J\n', res.E_cin(1));
    fprintf('  • Energía disipada total: %.2f J\n', res.E_dis(end));
    fprintf('  • Eficiencia de disipación: %.1f%\n', (res.E_dis(end)/res.E_cin(1))*100);
    
    % Estado general del diseño
    cumple_todo = res.cumple_fuerza && res.cumple_carrera && res.cumple_tiempo;
    fprintf('\n🎯 ESTADO GENERAL DEL DISEÑO:\n');
    if cumple_todo
        fprintf('  ✅ DISEÑO APROBADO - Cumple todas las especificaciones\n');
    else
        fprintf('  ❌ DISEÑO REQUIERE AJUSTES - No cumple todas las especificaciones\n');
    end
    
    if exist('mejora', 'var') && ~isempty(mejora)
        fprintf('\n🔧 MEJORAS IMPLEMENTADAS:\n');
        fprintf('  • Reducción de tiempo: %.2f s → %.2f s (%.1f%% mejora)\n', ...
        res.t_98, mejora.t_establecimiento, ((res.t_98 - mejora.t_establecimiento)/res.t_98)*100);
    end
    
    fprintf('\n=========================================================================\n');
end

function estado = evaluar_cumplimiento(cumple)
    if cumple
        estado = '✅ CUMPLE';
    else
        estado = '❌ NO CUMPLE';
    end
end
