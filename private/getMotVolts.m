function [V_B,V_C,V_D,V_E] = getMotVolts(p)
% This function outputs motor voltages that should be applied to each
% motor.
% p = Struct with at-least following fields
%   Vllmax = Maximum line to line voltage applicable to a motor, i.e.
%   battery voltage.
%   simNode = ROS2 node correspoing to this simulation

% Create a single time subscriber
persistent motVolts_sub
if isempty(motVolts_sub)
    motVolts_sub = ros2subscriber(p.simNode,"motVolts","std_msgs/Float32MultiArray","History","keeplast","Depth",1);
end

% Message format of fl_bl_br_fr is assumed and all the values should fall
% between 0 and Vllmax.
if isempty(motVolts_sub.LatestMessage)
    V_B = 0;
    V_C = 0;
    V_D = 0;
    V_E = 0;
elseif strcmp(motVolts_sub.LatestMessage.layout.dim.label,'fl_bl_br_fr') && motVolts_sub.LatestMessage.layout.dim.size == 4
    V_B = double(motVolts_sub.LatestMessage.data(1));
    V_C = double(motVolts_sub.LatestMessage.data(2));
    V_D = double(motVolts_sub.LatestMessage.data(3));
    V_E = double(motVolts_sub.LatestMessage.data(4));
else
    error("rcIn message contains invalid data.");
end
end