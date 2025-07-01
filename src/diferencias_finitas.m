function [t, u, u_dot, u_ddot, F_amort] = diferencias_finitas(parametros, k, c)
% DIFERENCIAS_FINITAS - Euler directo de m ü + c u̇ + k u = F_ext

    t_final = parametros.t_final;
    dt      = parametros.dt;
    t       = 0:dt:t_final;
    n       = numel(t);

    u       = zeros(n,1);
    u_dot   = zeros(n,1);
    u_ddot  = zeros(n,1);
    F_amort = zeros(n,1);

    u_dot(1) = parametros.v0;
    u_ddot(1)= (parametros.F_ext - c*u_dot(1) - k*u(1)) / parametros.m;
    F_amort(1)= c*abs(u_dot(1)) + k*abs(u(1));

    for i=2:n
        u_ddot(i-1) = (parametros.F_ext - c*u_dot(i-1) - k*u(i-1)) / parametros.m;
        u_dot(i)    = u_dot(i-1) + u_ddot(i-1)*dt;
        u(i)        = u(i-1)    + u_dot(i-1)*dt;
        F_amort(i)  = c*abs(u_dot(i)) + k*abs(u(i));
    end
    u_ddot(n) = (parametros.F_ext - c*u_dot(n) - k*u(n)) / parametros.m;
end