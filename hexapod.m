close all;
clear variables;

r = RigidBodyManipulator('urdf/hexapod.urdf');



controller = HexapodLegController(r);
sys_closedloop = feedback(r,controller);

%x0 = Point(r.getStateFrame);
x0 = zeros(6,1);
x0 = [1; -1; -1; 0; 0; 0];

v = r.constructVisualizer();
xtraj = simulate(sys_closedloop,[0 5], x0);
v.playback(xtraj);

