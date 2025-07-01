function [mejor_ajuste, velocidad, fuerza, error_ajuste] = ajuste_curva()
% AJUSTE_CURVA - Ajuste de curva fuerza vs velocidad por mínimos cuadrados
%
% Salidas:
%   mejor_ajuste  - Coeficientes del mejor polinomio
%   velocidad     - Vector de velocidades experimentales [m/s]
%   fuerza        - Vector de fuerzas experimentales [N]
%   error_ajuste  - Error RMS mínimo obtenido

    % Datos experimentales
    velocidad = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9];
    fuerza    = [1500,2200,2800,3300,3700,4000,4200,4300];

    grados = 1:4;
    errores = zeros(size(grados));
    coef = cell(size(grados));

    for i = grados
        coef{i} = polyfit(velocidad, fuerza, i);
        f_pred = polyval(coef{i}, velocidad);
        errores(i) = sqrt(mean((f_pred - fuerza).^2));
    end

    [error_ajuste, idx] = min(errores);
    mejor_ajuste = coef{idx};

    % (Opcional) mostrar ecuación y residuos…
end