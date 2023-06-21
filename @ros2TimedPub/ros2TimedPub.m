classdef ros2TimedPub < ros2publisher
    properties (SetAccess = private)
        lastT (1,1) double = 0
    end
    properties
        msg (1,1) struct
        dt (1,1) double
    end
    methods
        function obj = ros2TimedPub(node,topic,msgType,dt)
            arguments
                node (1,1) ros2node
                topic (1,1) string
                msgType (1,1) string
                dt (1,1) double
            end
            mustBeMember(msgType,ros2("msg","list"));
            obj = obj@ros2publisher(node,topic,msgType);
            obj.msg = ros2message(obj);
            obj.dt = dt;
        end
        function send(obj,t)
            if t >= (obj.lastT + obj.dt)
                send@ros2publisher(obj,obj.msg);
                if t >= (obj.lastT + 2*obj.dt)
                    obj.lastT = t;
                else
                    obj.lastT = obj.lastT + obj.dt;
                end
            end
        end
    end
end