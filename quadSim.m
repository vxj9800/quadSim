%% Load model parameters
quadParams;

%% Create a ROS2 node for this simulation %%
simNode = ros2node("quadSimNode",1);

%% Create ROS2 publishers %%
timePub = ros2TimedPub(simNode,"time","builtin_interfaces/Time",0.01);
posPub = ros2TimedPub(simNode,"pos","geometry_msgs/Vector3",0.1);
angVelPub = ros2TimedPub(simNode,"angVel","geometry_msgs/Vector3",0.01);
linAccPub = ros2TimedPub(simNode,"linAcc","geometry_msgs/Vector3",0.01);

%% Convert everything into a struct %%
params = v2struct();

%% Initial Conditions %%
% Coordinates
q1_3 = [0; 0; 0];
% e = [1; 0; 0; 0];
e = eul2quat([0 -pi/10 pi/10])';
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
% [~,Q] = ode45(@quadDer, [t t+dt], q, [], p);
% q = Q(end,:)';
% t = t + dt;

% Solve ODEs
Q = ode5(@quadDer, [t t+dt], q, p);
q = Q(end,:)';
t = t + dt;

% Publish the updated timestamp
p.timePub.msg.sec = int32(floor(t));
p.timePub.msg.nanosec = uint32((t - double(p.timePub.msg.sec))*1e9);
p.timePub.send(t);

% Publish the position
p.posPub.msg.x = q(1);
p.posPub.msg.y = q(2);
p.posPub.msg.z = q(3);
p.posPub.send(t);

% Publish gyro data
p.angVelPub.msg.x = q(15);
p.angVelPub.msg.y = q(16);
p.angVelPub.msg.z = q(17);
p.angVelPub.send(t);

% Publish accelerometer data
dx = quadDer(t,q,p);
dx = dx(12:14);
R_NA = e2R(q(4:7));
dx = R_NA'*dx;
p.linAccPub.msg.x = dx(1)/p.g;
p.linAccPub.msg.y = dx(2)/p.g;
p.linAccPub.msg.z = dx(3)/p.g;
p.linAccPub.send(t);

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
