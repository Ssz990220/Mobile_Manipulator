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

