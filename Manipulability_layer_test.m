manipulator = robot(0);
syms theta1 theta2 theta3 theta4 theta5 theta6;
params = [theta1 theta2 theta3 theta4 theta5 theta6];
a2=40; a3 = 10; d3=10;d4 = 30;
manipulator.Add_link(link(0,0,0,theta1));
manipulator.Add_link(link(-pi/2,0,0,theta2));
manipulator.Add_link(link(0,a2,d3,theta3));
manipulator.Add_link(link(-pi/2,a3,d4,theta4));
manipulator.Add_link(link(pi/2, 0, 0, theta5));
manipulator.Add_link(link(-pi/2,0,0,theta6));


alpha_i_1 =0;
a_i_1 = 0;
d_i = 40;
theta_i = 0;
target = [cos(theta_i) -sin(theta_i) 0 a_i_1;...
          sin(theta_i)*cos(alpha_i_1) cos(theta_i)*cos(alpha_i_1) -sin(alpha_i_1) -sin(alpha_i_1)*d_i;...
          sin(theta_i)*sin(alpha_i_1) cos(theta_i)*sin(alpha_i_1) cos(alpha_i_1) cos(alpha_i_1)*d_i;...
          0 0 0 1];
target(1,4)=10;
target(2,4)=10;


layer = Manipulability_layer(target,10,50);
m_map = layer.draw_map(manipulator,params);