function [th,tq] = propThTq(u,rho,propVel,p)
% u = Angular velocity of the motor/propeller in rad/sec
% rho = Density of the air.
% propVel = Translational velocity of the propeller in the direction of
% the axis of rotation.
% p = Struct containing at-least following fields
%   cThFitCoef = Curve fit parameters to convert advancement ratio (J) to
%   thrust coefficient (C_T).
%   cTqFitCoef = Curve fit parameters to convert advancement ratio (J) to
%   torque coefficient (C_Q).
%   propDia = Diameter of the propeller.

% This function depends on the UIUC dataset. First, a second order
% polynomial is fitted to the thrust coefficient and torque coefficient
% data. Those curve-fit coefficients are passed to this function to
% estimate the C_T and C_Q values based on the advancement ratio (J). From
% there, the thrust and the torque is calculated. One assumption here is
% that the air is stationary at all times. However, it should be easy to
% account for the air velocity as well. It is also assumed that the
% propeller rotates in such a way that it creates thrust to lift the
% quadcopter.

% Define conversion factor for rad/sec to revs/sec
radpsec2revpsec = 0.159154943;

% Return zero torque and thrust if the props are not rotating
if u == 0
    th = 0; tq = 0; return;
end

u = abs(u);

% Calculate advancement ratio
J = propVel / (u*radpsec2revpsec) / p.propDia;

% Calculate thrust and torque coefficient from curve-fit coefficients
% The coefficients should follow format given below
% y = x^2 * coef(1) + x * coef(2) + coef(3)
C_T = J^2 * p.cThFitCoef(1) + J * p.cThFitCoef(2) + p.cThFitCoef(3);
C_Q = J^2 * p.cTqFitCoef(1) + J * p.cTqFitCoef(2) + p.cTqFitCoef(3);

% Calculate the thrust and the torque
th = C_T * rho * (u*radpsec2revpsec)^2 * p.propDia^4;
tq = C_Q * rho * (u*radpsec2revpsec)^2 * p.propDia^5;
end