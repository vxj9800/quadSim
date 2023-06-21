function [V_B,V_C,V_D,V_E] = inputMixer(roll, pitch, yaw, thrust, p)
% This function mixes the desired roll, pitch, yaw and thrust values and
% outputs voltages for each motor of the Quad X configuration. The
% configuration assumes that motors B and E are in the front while motors C
% and D are in the back.

V_B = (thrust - pitch + roll + yaw)*p.Vllmax;
V_C = (thrust + pitch + roll - yaw)*p.Vllmax;
V_D = (thrust + pitch - roll + yaw)*p.Vllmax;
V_E = (thrust - pitch - roll - yaw)*p.Vllmax;

if V_B > p.Vllmax
    V_B = p.Vllmax;
elseif V_B < 0
    V_B = 0;
end

if V_C > p.Vllmax
    V_C = p.Vllmax;
elseif V_C < 0
    V_C = 0;
end

if V_D > p.Vllmax
    V_D = p.Vllmax;
elseif V_D < 0
    V_D = 0;
end

if V_E > p.Vllmax
    V_E = p.Vllmax;
elseif V_E < 0
    V_E = 0;
end

end