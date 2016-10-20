classdef HexapodLegController < DrakeSystem
    properties
        p
        step
    end
    methods
        function obj = HexapodLegController(robot)
            obj = obj@DrakeSystem(0,0,6,3,true,true);
            obj.p = robot;
            obj = obj.setInputFrame(robot.getStateFrame);
            obj = obj.setOutputFrame(robot.getInputFrame);
        end
        
        function u = output(obj,t,~,x)                                  
            u = [0;0;0];
            
            Q = [0.1,0,0,0,0,0;
                 0,100,0,0,0,0;
                 0,0,100,0,0,0;
                 0,0,0,0,0,0;
                 0,0,0,0,0,0;
                 0,0,0,0,0,0];
             
             R = [1,0,0;
                 0,1,0;
                 0,0,1];
            
            [A,B,C,D,x0dot,y0] = obj.p.linearize(t, x, u);
            [K,S] = lqr(A,B,Q,R);
           
           x - [0;0;0;0;0;0]
           u = -K*(x - [sin(t*2*pi);0;0;0;0;0])
        end
    end
end

