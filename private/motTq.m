function tq = motTq(u,V,p)
% u = current angular velocity of the motor in rad/sec
% V = Voltage applied
% p = Struct with at least following fields
%   motRll = Line-To-Line resistance of the motor winding
%   motKv = Motor Kv rating, i.e. RPM/V

% BLDC/PMSM motors can be delta/star wound. It is important to determine
% the winding type since that will affect the governing equations to a
% certain degree. Generally, hobby motor datasheets do not specify the
% winding type. However, it is noted at some places on the internet that
% the winding type of the RC hobby BLDC motors is generally delta, that
% achieves higher KV rating and lower thrust value. So, the delta type of
% winding is assumed here. The governing equations are taken from a youtube
% video available at https://www.youtube.com/watch?v=jrWDBkeOVQY&list=WL&index=27&ab_channel=IROS2021workshop%3AFromgearstodirectdrive
% It is also assumed here that dI/dt = 0 or L (winding inductance) is
% zero. This assumption makes the equations look very similar to the ones
% for the DC motor.

u = abs(u);

% Define conversion factor for rad/sec to revs/min
radpsec2revpmin = 9.5492968;

% Compute phase resistance from line to line resistance
Rphi = 1/2*p.motRll; % For Wye winding
Rphi = 3/2*p.motRll; % For Delta winding

% Back-EMF constant and torque constant are generally close to each other
% and can be computed from Kv value
Kt = sqrt(1/2)/p.motKv; % For Wye winding
Kt = sqrt(3/2)/p.motKv; % For Delta winding
Kb = Kt;

% Calculate the torque produced by the motor
tq = Kt*(V - Kb*u*radpsec2revpmin)/Rphi;
end