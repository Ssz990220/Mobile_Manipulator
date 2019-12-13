syms theta1 theta2 theta3 theta4 theta5 theta6
a2=40; a3 = 10; d3=10;d4 = 30;
link1 = link(0, 0, 0, theta1);
link2 = link(-pi/2,0,0,theta2);
link3 = link(0,a2,d3,theta3);
link4 = link(-pi/2,a3,d4,theta4);
link5 = link(pi/2,0,0,theta5);
link6 = link(-pi/2,0,0,theta6);
T0_6 = link1.T*link2.T*link3.T*link4.T*link5.T*link6.T;

px = T0_6(1,4);
py = T0_6(2,4);
pz = T0_6(3,4);
J = jacobian([px;py;pz],[theta1 theta2 theta3 theta4 theta5 theta6]);
j = subs(J,{theta1 theta2 theta3 theta4 theta5 theta6},{pi/3,pi/3,pi/3,pi/3,pi/3,pi/3});
sqrt(det(j*j'))