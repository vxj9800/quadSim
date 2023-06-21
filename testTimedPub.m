testNode = ros2node("testNode",1);
timePub = ros2TimedPublisher(testNode,"time","std_msgs/Float64",0.001);

tic

while true
    t = toc;

    % Publish the updated timestamp
    time_msg = ros2message(timePub);
    time_msg.data = t;
    send(timePub,time_msg,t);
    disp(t);
end
