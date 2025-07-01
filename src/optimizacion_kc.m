function [k_opt, c_opt, error_optimizacion] = optimizacion_kc(velocidad, fuerza, ajuste, parametros)
% OPTIMIZACION_KC - Busca k y c que minimizan RMS contra el polinomio “ajuste”
%
% Salidas:
%   k_opt               - Rigidez óptima [N/m]
%   c_opt               - Amortiguamiento óptimo [N·s/m]
%   error_optimizacion  - Error RMS alcanzado

    k_vals = linspace(5e4, 2e5, 50);
    c_vals = linspace(2e3, 1.5e4, 50);
    error_optimizacion = inf;

    for k = k_vals
        for c = c_vals
            f_pred = c*velocidad + k*0.1;           % modelo simplificado
            rms    = sqrt(mean((f_pred - fuerza).^2));
            if rms < error_optimizacion
                error_optimizacion = rms;
                k_opt = k;
                c_opt = c;
            end
        end
    end
end