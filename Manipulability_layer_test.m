%定义机器人
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

%目标位姿
alpha_i_1 =0;
a_i_1 = 0;
d_i = 20;
theta_i = 0;
target = [cos(theta_i) -sin(theta_i) 0 a_i_1;...
          sin(theta_i)*cos(alpha_i_1) cos(theta_i)*cos(alpha_i_1) -sin(alpha_i_1) -sin(alpha_i_1)*d_i;...
          sin(theta_i)*sin(alpha_i_1) cos(theta_i)*sin(alpha_i_1) cos(alpha_i_1) cos(alpha_i_1)*d_i;...
          0 0 0 1];

%定义可操作度图相关参数
edges_total =10;
resolution =10;
manipulability_map =zeros(3,(2*resolution+1)^2);
map = cell(2*resolution+1);
edge = edges_total/(2*resolution);  
for i = 1:(2*resolution+1)
    for j = 1:(2*resolution + 1)
        map{i,j} = [(resolution+1-i)*edge (-resolution-1+j)*edge];   %构建地图map，点的坐标为机器人base_coor下原点（target）的相对坐标[x y]
    end
end
Manipulability_equ = cal_manipulability(manipulator,params);

%画图
for i = 1:(2*resolution+1)
    for j = 1:(2*resolution+1)
        pose = target;
        x = map{i,j}(1);
        y = map{i,j}(2);
        pose(1,4)=x;
        pose(2,4)=y;
        thetas = inverse_kinematics_puma560_simple(pose);
        manipulability = subs(Manipulability_equ,params,thetas);
        manipulability_map((i-1)*(2*resolution+1)+j,:)=[x,y,manipulability];
    end
end    
scatter3(manipulability_map(:,1),manipulability_map(:,2),manipulability_map(:,3))