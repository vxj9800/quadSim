function dq = quadDer(t,q,p)
% t = ODE timestamp
% q = System states for time t
% p = Structure containing system parameters
% Split generalized coordinates and speeds
u = q(12:end); q = q(1:11);

q1_3 = q(1:3);
e = q(4:7);
q4_7 = q(8:11);

u1_3 = u(1:3);
w = u(4:6);
u4_7 = u(7:10);

% Normalize euler parameters and compute the euler rates
e = e./norm(e);
ed = omega2edot(w,e);

% Compute the thrust and torque from the propeller
rho = 1; propVel = 0;
[th,tq] = propThTq(u4_7(1),rho,propVel,p); % Thrust and torque for motor B
fB = th; tB = -tq;
[th,tq] = propThTq(u4_7(2),rho,propVel,p); % Thrust and torque for motor C
fC = th; tC = -tq;
[th,tq] = propThTq(u4_7(3),rho,propVel,p); % Thrust and torque for motor D
fD = th; tD = -tq;
[th,tq] = propThTq(u4_7(4),rho,propVel,p); % Thrust and torque for motor E
fE = th; tE = -tq;
% fB = 0.00; fC = 0.00; fD = 0.00; fE = 0.00; % Assume that there is no propeller for now

% Compute motor torques based on the control input
[V_B,V_C,V_D,V_E] = getMotVolts(p);
tB = tB + motTq(u4_7(1),V_B,p);
tC = tC + motTq(u4_7(2),V_C,p);
tD = tD + motTq(u4_7(3),V_D,p);
tE = tE + motTq(u4_7(4),V_E,p);
% tB = t*0.02; tC = t*0.01; tD = t*0.02; tE = t*0.01; % Torques are functions of time for now

% Compute mass matrix and sum of forces
COEF = coef([q;u],p.g,p.pB,p.pC,p.pD,p.pE,[p.mA; p.mB; p.mC; p.mD; p.mE],...
            p.A_I_AA,p.B_I_BB,p.C_I_CC,p.D_I_DD,p.E_I_EE,[fB; fC; fD; fE],...
            [tB; tC; tD; tE]);
RHS = rhs([q;u],p.g,p.pB,p.pC,p.pD,p.pE,[p.mA; p.mB; p.mC; p.mD; p.mE],...
          p.A_I_AA,p.B_I_BB,p.C_I_CC,p.D_I_DD,p.E_I_EE,[fB; fC; fD; fE],...
          [tB; tC; tD; tE]);

% Compute generalized accelerations
ud = COEF\RHS;

% Put the state derivatives together
dq = [u1_3; ed; u4_7; ud];
end