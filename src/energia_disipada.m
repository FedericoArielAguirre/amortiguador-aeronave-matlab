function res = energia_disipada(t, u, u_dot, F_amort, parametros, k, c)
% ENERGIA_DISIPADA - Calcula E_cin, E_pot, E_dis y métricas de desempeño
%
% Salida:
%   res (struct) con campos:
%     .E_cin, .E_pot, .E_dis, .E_tot
%     .F_max, .x_max, .t_98
%     .cumple_fuerza, .cumple_carrera, .cumple_tiempo

    m  = parametros.m;
    dt = t(2)-t(1);

    res.E_cin = 0.5*m*u_dot.^2;
    res.E_pot = 0.5*k*u.^2;
    res.E_dis = cumsum(c*u_dot.^2)*dt;
    res.E_tot = res.E_cin + res.E_pot + res.E_dis;

    % Métricas
    res.F_max = max(F_amort);
    res.x_max = max(abs(u));
    % t_98 = tiempo en que u alcanza 2% de valor final de amortiguación
    u_end = u(end);
    idx98 = find(abs(u - u_end)<=0.02*abs(u_end),1,'last');
    res.t_98 = t(idx98);

    res.cumple_fuerza   = (res.F_max <= parametros.f_max);
    res.cumple_carrera  = (res.x_max <= parametros.carrera_max);
    res.cumple_tiempo   = (res.t_98 <= parametros.t_amort_max);
    % Para compatibilidad con generar_reporte_final:
    res.carrera_max = res.x_max;
end