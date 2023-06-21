function edot = omega2edot(omega,e)
% This function calculates euler speeds from angular speed in body attached
% frame and euler parameters using the equation edot =
% 0.5*E*eta'*omega
arguments
    omega (3,1) double
    e (4,1) double
end

edot = calc_L(e)'*omega;
end