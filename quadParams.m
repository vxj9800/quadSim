%% Model Parameter Samples %%
% Frame: https://speedyfpv.com/products/truexs-stretched-220mm-fpv-racing-drone-frame-kit-for-5-propellers?variant=39876517363885
% Motors: https://shop.iflight-rc.com/xing-2205-fpv-nextgen-motor-black-pro1521?search=2205
% Battery: https://www.amazon.com/Socokin-Battery-3300mAh-Vehicles-Trucks/dp/B086GTSPQN/ref=sr_1_28?crid=21QGKY7B7764X&keywords=6s+lipo&qid=1686777293&sprefix=6s+lipo%2Caps%2C142&sr=8-28&ufe=app_do%3Aamzn1.fos.18630bbb-fcbb-42f8-9767-857e17e03685
% Propellers: https://www.getfpv.com/lumenier-5x3-5-2-blade-propeller-set-of-4-green.html

%% Constants for motor positions %%
pB = [0.08; 0.08; 0.015]; % pB = [lB; wB; hB];
pC = [-0.08; 0.08; 0.015]; % pC = [lC; wC; hC];
pD = [-0.08; -0.08; 0.015]; % pD = [lD; wD; hD];
pE = [0.08; -0.08; 0.015]; % pE = [lE; wE; hE];

%% Mass and Inertia %%
g = 9.8; % Gravitational Constant
mA = 0.155 + 4*(0.021*0.9) + 0.5 + 0.1; % Frame + 4*Motor Stator + Battery + Circuit boards
mB = (0.021*0.1) + 0.0028; % Motor rotor + Prop
mC = (0.021*0.1) + 0.0028; % Motor rotor + Prop
mD = (0.021*0.1) + 0.0028; % Motor rotor + Prop
mE = (0.021*0.1) + 0.0028; % Motor rotor + Prop
A_I_AA = diag([1/12*0.5*(0.042^2 + 0.043^2),... % Inertia dyadic of the battery
               1/12*0.5*(0.137^2 + 0.043^2),... % as a cuboid of dimension 137mm X 42mm X 43mm
               1/12*0.5*(0.137^2 + 0.042^2)])...
        +simprot(3,pi/4)...
        *diag([1/12*0.0775*(0.030^2 + 0.005^2),... % Inertia dyadic of half the frame
               1/12*0.0775*(0.220^2 + 0.005^2),... % as a cuboid of dimension 220mm X 30mm X 5mm
               1/12*0.0775*(0.220^2 + 0.020^2)])...
        *simprot(3,pi/4)'...
        +simprot(3,-pi/4)...
        *diag([1/12*0.0775*(0.030^2 + 0.005^2),... % Inertia dyadic of half the frame
               1/12*0.0775*(0.220^2 + 0.005^2),... % as a cuboid of dimension 220mm X 30mm X 5mm
               1/12*0.0775*(0.220^2 + 0.020^2)])...
        *simprot(3,-pi/4)';
B_I_BB = diag([1/12*(0.021*0.1)*(3*(0.0286^2 + 0.025^2) + 0.01^2),... % Inertia dyadic of motor rotor as a hollow cylinder with
               1/12*(0.021*0.1)*(3*(0.0286^2 + 0.025^2) + 0.01^2),... % outer radius of 28.1mm and inner of 25mm and height of 10mm
               1/2*(0.021*0.1)*(0.0286^2 + 0.025^2)])...
        +diag([1/12*0.0028*(0.001^2 + 0.01^2),... % Inertia dyadic of propeller
               1/12*0.0028*(0.001^2 + 0.127^2),... % as a cuboid of dimension 127mm X 10mm X 1mm
               1/12*0.0028*(0.01^2 + 0.127^2)]);
C_I_CC = B_I_BB;
D_I_DD = B_I_BB;
E_I_EE = B_I_BB;

%% Motor Properties %%
Vllmax = 16.8; % Max line to line voltage for the motor
motRll = 0.07241; % Line to line resistance
motKv = 2400; % Kv rating of the motor, i.e. rpm/V

%% Get propeller data and create a curvefit %%
propDia = 0.127; % Propeller Diameter
[cThFitCoef,cTqFitCoef] = calcPropDataFit("propData\");