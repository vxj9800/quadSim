%% Load model parameters
quadParams;

%% Create a ROS2 node for this simulation %%
simNode = ros2node("quadSimNode",1);

%% Create ROS2 publishers %%
timePub = ros2TimedPub(simNode,"time","builtin_interfaces/Time",0.01);
posPub = ros2TimedPub(simNode,"gpsRaw","geometry_msgs/Vector3",0.1);
attPub = ros2TimedPub(simNode,"imu/attitude","geometry_msgs/Vector3",0.001);
angVelPub = ros2TimedPub(simNode,"imu/gyroRaw","geometry_msgs/Vector3",0.001);
linAccPub = ros2TimedPub(simNode,"imu/acclRaw","geometry_msgs/Vector3",0.001);

%% Convert everything into a struct %%
params = v2struct();

%% Initial Conditions %%
% Coordinates
q1_3 = [0; 0; 0];
e = [1; 0; 0; 0];
% e = eul2quat([0 -pi/10 pi/10])';
q4_7 = [7*pi/4; pi/4; 3*pi/4; 5*pi/4];
q0 = [q1_3; e; q4_7];
% Speeds
u1_3 = [0; 0; 0];
w = [0; 0; 0];
u4_7 = [0; 0; 0; 0];
u0 = [u1_3; w; u4_7];

%% Create a timer object %%
dt = 0.01; % Simulation frame period
t = timer("TimerFcn",@(stepSize,initCond,par) timedFunction(dt,[q0;u0],params),...
          "BusyMode","drop","ExecutionMode","fixedRate","Period",dt);
start(t);
input('Press ''Enter'' to continue...','s');
stop(t);
delete(t);

%% Clear-up the workspace %%
clear; close all; % Standard clear and close
% Clear persistent variables
clear getControlInput
clear timedFunction

%% Timed function to run the simulation %%
function timedFunction(dt,initCond,p)
% Create persistant variables
persistent q t % System States
if isempty(q)
    q = initCond;
    t = 0;
end

% % Solve ODEs
% [T,Q] = ode45(@quadDer, t:p.linAccPub.dt:t+dt, q, [], p);

% Solve ODEs
T = t:p.linAccPub.dt:t+dt;
Q = ode5(@quadDer, T, q, p);

% Publish IMU data
for i = 2:length(T)
    % Publish attitude
    rpy = quat2eul(Q(i,4:7));
    p.attPub.msg.x = rpy(3);
    p.attPub.msg.y = rpy(2);
    p.attPub.msg.z = rpy(1);
    p.attPub.send(T(i));
    
    % Publish gyro data
    p.angVelPub.msg.x = Q(i,15);
    p.angVelPub.msg.y = Q(i,16);
    p.angVelPub.msg.z = Q(i,17);
    p.angVelPub.send(T(i));
    
    % Publish accelerometer data
    dx = quadDer(T(i),Q(i,:)',p);
    dx = dx(12:14);
    dx = dx + [0; 0; -p.g];
    R_NA = e2R(Q(i,4:7));
    dx = R_NA'*dx;
    p.linAccPub.msg.x = dx(1)/p.g;
    p.linAccPub.msg.y = dx(2)/p.g;
    p.linAccPub.msg.z = dx(3)/p.g;
    p.linAccPub.send(T(i));
end

% Update the last state
q = Q(end,:)';
t = t + dt;

% Correct for euler parameter drift
q(4:7) = q(4:7)./norm(q(4:7));

% Limit motor angles in 0-2pi range
q(8:11) = mod(q(8:11),2*pi);

% Publish the updated timestamp
p.timePub.msg.sec = int32(floor(t));
p.timePub.msg.nanosec = uint32((t - double(p.timePub.msg.sec))*1e9);
p.timePub.send(t);

% Publish the position
p.posPub.msg.x = q(1);
p.posPub.msg.y = q(2);
p.posPub.msg.z = q(3);
p.posPub.send(t);

% Generate animation frame
persistent f % Figure handle
if isempty(f)
    f = figure("WindowStyle","docked");
    % f = figure("WindowState","normal");
    set(f, 'MenuBar', 'none');
    set(f, 'ToolBar', 'none');
end
persistent qBodyPlot % line plot object
if isempty(qBodyPlot)
    % Create first plot
    [x,y,z] = q2AnimLines(q,p.pB,p.pC,p.pD,p.pE,p.propDia);
    qBodyPlot = plot3(x(:,1:end-4),y(:,1:end-4),z(:,1:end-4),'b',x(:,end-3:end),y(:,end-3:end),z(:,end-3:end),'k','LineWidth',1.5);
    set(gca,"projection","perspective");
    grid on
    axis equal;
    axis([x(1)-5*p.propDia x(1)+5*p.propDia y(1)-5*p.propDia y(1)+5*p.propDia z(1)-5*p.propDia z(1)+5*p.propDia]) 
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
else
    % Update animation plot
    [x,y,z] = q2AnimLines(q,p.pB,p.pC,p.pD,p.pE,p.propDia);
    for j = 1:size(x,2)
        qBodyPlot(j).XData = x(:,j);
        qBodyPlot(j).YData = y(:,j);
        qBodyPlot(j).ZData = z(:,j);
    end
    axis([x(1)-5*p.propDia x(1)+5*p.propDia y(1)-5*p.propDia y(1)+5*p.propDia z(1)-5*p.propDia z(1)+5*p.propDia]);
    drawnow limitrate;
end
disp(t);

end
