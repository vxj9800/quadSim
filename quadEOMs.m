clear; clc;

%% Define Symbolic Variables %%
% Define generalized coordinates
syms q1 q2 q3 real % 3 coordinates for the position of the main body
syms e0 e1 e2 e3 real % 4 euler parameters for the orientation of the body
syms q4 q5 q6 q7 real % 4 coordinates for the rotation of the motors w.r.t. the body

% Define generalized speeds
syms u1 u2 u3 real % 3 speeds for the translational velocity of the main body
syms w1 w2 w3 real % 3 speeds for the angular velocity of the main body
syms u4 u5 u6 u7 real % 4 speeds for the angular velocity of the motors w.r.t. the body

% Define generalized accelerations
syms ud1 ud2 ud3 ud4 ud5 ud6 ud7 real % 7 derivatives of "u"s
syms wd1 wd2 wd3 real % 3 derivatives of "w"s

% Define other necessary constants
syms g real % Gravitational Constant
syms lB wB hB lC wC hC lD wD hD lE wE hE real % Constants to define position of the motor rotors
syms pDia % Propeller diameter

% Define vectors for simplification of the math
q1_3 = [q1; q2; q3];
e = [e0; e1; e2; e3];
q4_7 = [q4; q5; q6; q7];
q = [q1_3; e; q4_7];

u1_3 = [u1; u2; u3];
w = [w1; w2; w3];
u4_7 = [u4; u5; u6; u7];
u = [u1_3; w; u4_7];

ud1_3 = [ud1; ud2; ud3];
wd = [wd1; wd2; wd3];
ud4_7 = [ud4; ud5; ud6; ud7];
ud = [ud1_3; wd; ud4_7];

%% Position and Orientation %%
R_NA = e2R(e); A_P_NA = R_NA'*q1_3;
R_AB = simprot(3,q4); A_P_AB = [lB; wB; hB];
R_AC = simprot(3,q5); A_P_AC = [lC; wC; hC];
R_AD = simprot(3,q6); A_P_AD = [lD; wD; hD];
R_AE = simprot(3,q7); A_P_AE = [lE; wE; hE];

%% Velocities %%
A_W_NA = w; A_V_NA = R_NA'*u1_3;
A_W_AB = [0; 0; u4]; A_W_NB = A_W_NA + A_W_AB; A_V_NB = A_V_NA + skew(A_P_AB)'*A_W_NA;
A_W_AC = [0; 0; u5]; A_W_NC = A_W_NA + A_W_AC; A_V_NC = A_V_NA + skew(A_P_AC)'*A_W_NA;
A_W_AD = [0; 0; u6]; A_W_ND = A_W_NA + A_W_AD; A_V_ND = A_V_NA + skew(A_P_AD)'*A_W_NA;
A_W_AE = [0; 0; u7]; A_W_NE = A_W_NA + A_W_AE; A_V_NE = A_V_NA + skew(A_P_AE)'*A_W_NA;

%% Partial Velocities %%
dA_V_NA_du = jacobian(A_V_NA,u); dA_W_NA_du = jacobian(A_W_NA,u);
dA_V_NB_du = jacobian(A_V_NB,u); dA_W_NB_du = jacobian(A_W_NB,u);
dA_V_NC_du = jacobian(A_V_NC,u); dA_W_NC_du = jacobian(A_W_NC,u);
dA_V_ND_du = jacobian(A_V_ND,u); dA_W_ND_du = jacobian(A_W_ND,u);
dA_V_NE_du = jacobian(A_V_NE,u); dA_W_NE_du = jacobian(A_W_NE,u);

%% Accelerations %%
A_Alf_NA = wd; A_Vd_NA = R_NA'*ud1_3;
A_Alf_NB = A_Alf_NA + [0; 0; ud4] + skew(A_W_AB)'*A_W_NA; A_Vd_NB = A_Vd_NA + skew(A_P_AB)'*A_Alf_NA + skew(A_W_NA)*skew(A_P_AB)'*A_W_NA;
A_Alf_NC = A_Alf_NA + [0; 0; ud5] + skew(A_W_AC)'*A_W_NA; A_Vd_NC = A_Vd_NA + skew(A_P_AC)'*A_Alf_NA + skew(A_W_NA)*skew(A_P_AC)'*A_W_NA;
A_Alf_ND = A_Alf_NA + [0; 0; ud6] + skew(A_W_AD)'*A_W_NA; A_Vd_ND = A_Vd_NA + skew(A_P_AD)'*A_Alf_NA + skew(A_W_NA)*skew(A_P_AD)'*A_W_NA;
A_Alf_NE = A_Alf_NA + [0; 0; ud7] + skew(A_W_AE)'*A_W_NA; A_Vd_NE = A_Vd_NA + skew(A_P_AE)'*A_Alf_NA + skew(A_W_NA)*skew(A_P_AE)'*A_W_NA;

%% Mass and Inertia %%
syms mA mB mC mD mE % Mass for each body
syms A_I_AA B_I_BB C_I_CC D_I_DD E_I_EE [3 3] % Inertia Tensor for each body

%% Rate of Change of Angular Momentum
A_Hd_AA = A_I_AA*A_Alf_NA + skew(A_W_NA)*A_I_AA*A_W_NA;
A_Hd_BB = R_AB*B_I_BB*R_AB'*A_Alf_NB + skew(A_W_NB)*R_AB*B_I_BB*R_AB'*A_W_NB;
A_Hd_CC = R_AC*C_I_CC*R_AC'*A_Alf_NC + skew(A_W_NC)*R_AC*C_I_CC*R_AC'*A_W_NC;
A_Hd_DD = R_AD*D_I_DD*R_AD'*A_Alf_ND + skew(A_W_ND)*R_AD*D_I_DD*R_AD'*A_W_ND;
A_Hd_EE = R_AE*E_I_EE*R_AE'*A_Alf_NE + skew(A_W_NE)*R_AE*E_I_EE*R_AE'*A_W_NE;

%% Forces and Moments
syms fB fC fD fE % Thrust forces from the propeller
syms tB tC tD tE % Torques generated by the motors
A_F_A = R_NA'*[0; 0; -mA*g]; A_M_AA = [0; 0; tB - tC + tD - tE];
A_F_B = R_NA'*[0; 0; -mB*g] + [0; 0; fB]; A_M_BB = [0; 0; -tB];
A_F_C = R_NA'*[0; 0; -mC*g] + [0; 0; fC]; A_M_CC = [0; 0; tC];
A_F_D = R_NA'*[0; 0; -mD*g] + [0; 0; fD]; A_M_DD = [0; 0; -tD];
A_F_E = R_NA'*[0; 0; -mE*g] + [0; 0; fE]; A_M_EE = [0; 0; tE];

%% Equations of Motion %%
F = dA_V_NA_du'*A_F_A + dA_W_NA_du'*A_M_AA...
    + dA_V_NB_du'*A_F_B + dA_W_NB_du'*A_M_BB...
    + dA_V_NC_du'*A_F_C + dA_W_NC_du'*A_M_CC...
    + dA_V_ND_du'*A_F_D + dA_W_ND_du'*A_M_DD...
    + dA_V_NE_du'*A_F_E + dA_W_NE_du'*A_M_EE;
Fs = mA*dA_V_NA_du'*A_Vd_NA + dA_W_NA_du'*A_Hd_AA...
    + mB*dA_V_NB_du'*A_Vd_NB + dA_W_NB_du'*A_Hd_BB...
    + mC*dA_V_NC_du'*A_Vd_NC + dA_W_NC_du'*A_Hd_CC...
    + mD*dA_V_ND_du'*A_Vd_ND + dA_W_ND_du'*A_Hd_DD...
    + mE*dA_V_NE_du'*A_Vd_NE + dA_W_NE_du'*A_Hd_EE;
eqns = Fs == F;
[COEF,RHS] = equationsToMatrix(eqns,ud);

%% Generate function for COEF matrix %%
% Generate the function
matlabFunction(COEF,'file','./private/coef.m',...
               'Vars',{ ...
                    [q;u], ...
                    g, ...
                    A_P_AB, ...
                    A_P_AC, ...
                    A_P_AD, ...
                    A_P_AE, ...
                    [mA;mB;mC;mD;mE], ...
                    A_I_AA, B_I_BB, C_I_CC, D_I_DD, E_I_EE, ...
                    [fB; fC; fD; fE], ...
                    [tB; tC; tD; tE] ...
                    }, ...
               'Comments',[ ...
                   "    x = [q;u]";
                   "    g = Gravitational Constant";
                   "    A_P_AB = [lB;wB;hB]";
                   "    A_P_AC = [lC;wC;hC]";
                   "    A_P_AD = [lD;wD;hD]";
                   "    A_P_AE = [lE;wE;hE]";
                   "    mVals = [mA;mB;mC;mD;mE]";
                   "    A_I_AA = Inertia Tensor for body A";
                   "    B_I_BB = Inertia Tensor for body B";
                   "    C_I_CC = Inertia Tensor for body C";
                   "    D_I_DD = Inertia Tensor for body D";
                   "    E_I_EE = Inertia Tensor for body E";
                   "    fVals = [fB; fC; fD; fE]";
                   "    tVals = [tB; tC; tD; tE]" ...
                   ] ...
              );
% Replace inX input variable names with meaningful ones
mFileText = readlines("./private/coef.m");
mFileText = strrep(mFileText, "in10", "C_I_CC");
mFileText = strrep(mFileText, "in11", "D_I_DD");
mFileText = strrep(mFileText, "in12", "E_I_EE");
mFileText = strrep(mFileText, "in13", "fVals");
mFileText = strrep(mFileText, "in14", "tVals");
mFileText = strrep(mFileText, "in1", "x");
mFileText = strrep(mFileText, "in3", "A_P_AB");
mFileText = strrep(mFileText, "in4", "A_P_AC");
mFileText = strrep(mFileText, "in5", "A_P_AD");
mFileText = strrep(mFileText, "in6", "A_P_AE");
mFileText = strrep(mFileText, "in7", "mVals");
mFileText = strrep(mFileText, "in8", "A_I_AA");
mFileText = strrep(mFileText, "in9", "B_I_BB");
writelines(mFileText,"./private/coef.m");

%% Generate function for RHS vector %%
% Generate the function
matlabFunction(RHS,'file','./private/rhs.m',...
               'Vars',{ ...
                    [q;u], ...
                    g, ...
                    A_P_AB, ...
                    A_P_AC, ...
                    A_P_AD, ...
                    A_P_AE, ...
                    [mA;mB;mC;mD;mE], ...
                    A_I_AA, B_I_BB, C_I_CC, D_I_DD, E_I_EE, ...
                    [fB; fC; fD; fE], ...
                    [tB; tC; tD; tE] ...
                    }, ...
               'Comments',[ ...
                   "    x = [q;u]";
                   "    g = Gravitational Constant";
                   "    A_P_AB = [lB;wB;hB]";
                   "    A_P_AC = [lC;wC;hC]";
                   "    A_P_AD = [lD;wD;hD]";
                   "    A_P_AE = [lE;wE;hE]";
                   "    mVals = [mA;mB;mC;mD;mE]";
                   "    A_I_AA = Inertia Tensor for body A";
                   "    B_I_BB = Inertia Tensor for body B";
                   "    C_I_CC = Inertia Tensor for body C";
                   "    D_I_DD = Inertia Tensor for body D";
                   "    E_I_EE = Inertia Tensor for body E";
                   "    fVals = [fB; fC; fD; fE]";
                   "    tVals = [tB; tC; tD; tE]" ...
                   ] ...
              );
% Replace inX input variable names with meaningful ones
mFileText = readlines("./private/rhs.m");
mFileText = strrep(mFileText, "in10", "C_I_CC");
mFileText = strrep(mFileText, "in11", "D_I_DD");
mFileText = strrep(mFileText, "in12", "E_I_EE");
mFileText = strrep(mFileText, "in13", "fVals");
mFileText = strrep(mFileText, "in14", "tVals");
mFileText = strrep(mFileText, "in1", "x");
mFileText = strrep(mFileText, "in3", "A_P_AB");
mFileText = strrep(mFileText, "in4", "A_P_AC");
mFileText = strrep(mFileText, "in5", "A_P_AD");
mFileText = strrep(mFileText, "in6", "A_P_AE");
mFileText = strrep(mFileText, "in7", "mVals");
mFileText = strrep(mFileText, "in8", "A_I_AA");
mFileText = strrep(mFileText, "in9", "B_I_BB");
writelines(mFileText,"./private/rhs.m");

%% Generate function for computing position vectors %%
% Define Variables for Output
N_P_NA = R_NA*A_P_NA;
N_P_NB_base = R_NA*(A_P_NA + [A_P_AB(1:2); 0]);
N_P_NB = R_NA*(A_P_NA + A_P_AB);
N_P_NB_prop1 = R_NA*(A_P_NA + A_P_AB + R_AB*[pDia/2; 0; 0]);
N_P_NB_prop2 = R_NA*(A_P_NA + A_P_AB + R_AB*[-pDia/2; 0; 0]);
N_P_NC_base = R_NA*(A_P_NA + [A_P_AC(1:2); 0]);
N_P_NC = R_NA*(A_P_NA + A_P_AC);
N_P_NC_prop1 = R_NA*(A_P_NA + A_P_AC + R_AC*[pDia/2; 0; 0]);
N_P_NC_prop2 = R_NA*(A_P_NA + A_P_AC + R_AC*[-pDia/2; 0; 0]);
N_P_ND_base = R_NA*(A_P_NA + [A_P_AD(1:2); 0]);
N_P_ND = R_NA*(A_P_NA + A_P_AD);
N_P_ND_prop1 = R_NA*(A_P_NA + A_P_AD + R_AD*[pDia/2; 0; 0]);
N_P_ND_prop2 = R_NA*(A_P_NA + A_P_AD + R_AD*[-pDia/2; 0; 0]);
N_P_NE_base = R_NA*(A_P_NA + [A_P_AE(1:2); 0]);
N_P_NE = R_NA*(A_P_NA + A_P_AE);
N_P_NE_prop1 = R_NA*(A_P_NA + A_P_AE + R_AE*[pDia/2; 0; 0]);
N_P_NE_prop2 = R_NA*(A_P_NA + A_P_AE + R_AE*[-pDia/2; 0; 0]);
x = [N_P_NA(1) N_P_NB_base(1);
     N_P_NA(1) N_P_NC_base(1);
     N_P_NA(1) N_P_ND_base(1);
     N_P_NA(1) N_P_NE_base(1);
     N_P_NB_base(1) N_P_NB(1);
     N_P_NC_base(1) N_P_NC(1);
     N_P_ND_base(1) N_P_ND(1);
     N_P_NE_base(1) N_P_NE(1);
     N_P_NB_prop1(1) N_P_NB_prop2(1);
     N_P_NC_prop1(1) N_P_NC_prop2(1);
     N_P_ND_prop1(1) N_P_ND_prop2(1);
     N_P_NE_prop1(1) N_P_NE_prop2(1);]';
y = [N_P_NA(2) N_P_NB_base(2);
     N_P_NA(2) N_P_NC_base(2);
     N_P_NA(2) N_P_ND_base(2);
     N_P_NA(2) N_P_NE_base(2);
     N_P_NB_base(2) N_P_NB(2);
     N_P_NC_base(2) N_P_NC(2);
     N_P_ND_base(2) N_P_ND(2);
     N_P_NE_base(2) N_P_NE(2);
     N_P_NB_prop1(2) N_P_NB_prop2(2);
     N_P_NC_prop1(2) N_P_NC_prop2(2);
     N_P_ND_prop1(2) N_P_ND_prop2(2);
     N_P_NE_prop1(2) N_P_NE_prop2(2);]';
z = [N_P_NA(3) N_P_NB_base(3);
     N_P_NA(3) N_P_NC_base(3);
     N_P_NA(3) N_P_ND_base(3);
     N_P_NA(3) N_P_NE_base(3);
     N_P_NB_base(3) N_P_NB(3);
     N_P_NC_base(3) N_P_NC(3);
     N_P_ND_base(3) N_P_ND(3);
     N_P_NE_base(3) N_P_NE(3);
     N_P_NB_prop1(3) N_P_NB_prop2(3);
     N_P_NC_prop1(3) N_P_NC_prop2(3);
     N_P_ND_prop1(3) N_P_ND_prop2(3);
     N_P_NE_prop1(3) N_P_NE_prop2(3);]';
% Generate the function
matlabFunction(x, y, z, ...
               'file','./private/q2AnimLines.m',...
               'Vars',{ ...
                    q, ...
                    A_P_AB, ...
                    A_P_AC, ...
                    A_P_AD, ...
                    A_P_AE, ...
                    pDia, ...
                    }, ...
               'Comments',[ ...
                   "    xi = [qi,ui] -> i'th state values"; ...
                   "    A_P_AB = [lB;wB;hB]"; ...
                   "    A_P_AC = [lC;wC;hC]"; ...
                   "    A_P_AD = [lD;wD;hD]"; ...
                   "    A_P_AE = [lE;wE;hE]"; ...
                   "    pDia = Propeller Diameter" ...
                   ] ...
              );
% Replace inX input variable names with meaningful ones
mFileText = readlines("./private/q2AnimLines.m");
mFileText = strrep(mFileText, "in1", "xi");
mFileText = strrep(mFileText, "in2", "A_P_AB");
mFileText = strrep(mFileText, "in3", "A_P_AC");
mFileText = strrep(mFileText, "in4", "A_P_AD");
mFileText = strrep(mFileText, "in5", "A_P_AE");
writelines(mFileText,"./private/q2AnimLines.m");