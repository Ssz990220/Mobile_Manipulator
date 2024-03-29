function thetas = inverse_kinematics_puma560(target_pose_T)
px = target_pose_T(1,4);
py = target_pose_T(2,4);
pz = target_pose_T(3,4);
r11 = target_pose_T(1,1);
r13 = target_pose_T(1,3);
r23 = target_pose_T(2,3);
r33 = target_pose_T(3,3);
r31 = target_pose_T(3,1);
r21 = target_pose_T(2,1);
a2=40; a3 = 10; d3=10;d4 = 30;
theta11=atan2(py,px)-atan2(d3,sqrt(px^2+py^2-d3^2));
theta12=atan2(py,px)-atan2(d3,-sqrt(px^2+py^2-d3^2));
k = (px^2+py^2+pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
theta31  = atan2(a3,d4)-atan2(k,sqrt(a3^2+d4^2-k^2));
theta32  = atan2(a3,d4)-atan2(k,-sqrt(a3^2+d4^2-k^2));
%theta2-1 theta1 = theta11 theta3 = theta31
c1 = cos(theta11);
s1 = sin(theta11);
c3 = cos(theta31);
s3 = sin(theta31);
theta23 = atan2(((-a3-a2*c3)*pz+(c1*px+s1*py)*(a2*s3-d4)),((a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py)));
c23 = cos(theta23);
s23 = sin(theta23);
theta2_1 = theta23-theta31;
theta4_1 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
c4 = cos(theta4_1);      
s4 = sin(theta4_1);
s5 = -r13*(c1*c23*c4+s1*s4)-r23*(s1*c23*c4-c1*s4)+r33*(s23*c4);
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5_1 = atan2(s5,c5);
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*s23*s4;
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*c4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6_1 = atan2(s6,c6);
%theta2-2 theta1 = theta11 theta3 = theta32
c1 = cos(theta11);
s1 = sin(theta11);
c3 = cos(theta32);
s3 = sin(theta32);
theta23 = atan2(((-a3-a2*c3)*pz+(c1*px+s1*py)*(a2*s3-d4)),((a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py)));
c23 = cos(theta23);
s23 = sin(theta23);
theta2_2 = theta23-theta32;
theta4_2 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
c4 = cos(theta4_2);      
s4 = sin(theta4_2);
s5 = -r13*(c1*c23*c4+s1*s4)-r23*(s1*c23*c4-c1*s4)+r33*(s23*c4);
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5_2 = atan2(s5,c5);
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*s23*s4;
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*c4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6_2 = atan2(s6,c6);
%theta2-3 theta1 = theta12 theta3 = theta31
c1 = cos(theta12);
s1 = sin(theta12);
c3 = cos(theta31);
s3 = sin(theta31);
theta23 = atan2(((-a3-a2*c3)*pz+(c1*px+s1*py)*(a2*s3-d4)),((a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py)));
c23 = cos(theta23);
s23 = sin(theta23);
theta2_3 = theta23-theta31;
theta4_3 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
c4 = cos(theta4_3);      
s4 = sin(theta4_3);
s5 = -r13*(c1*c23*c4+s1*s4)-r23*(s1*c23*c4-c1*s4)+r33*(s23*c4);
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5_3 = atan2(s5,c5);
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*s23*s4;
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*c4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6_3 = atan2(s6,c6);
%theta2-3 theta1 = theta12 theta3 = theta32
c1 = cos(theta12);
s1 = sin(theta12);
c3 = cos(theta32);
s3 = sin(theta32);
theta23 = atan2(((-a3-a2*c3)*pz+(c1*px+s1*py)*(a2*s3-d4)),((a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py)));
c23 = cos(theta23);
s23 = sin(theta23);
theta2_4 = theta23-theta32;
theta4_4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
c4 = cos(theta4_4);      
s4 = sin(theta4_4);
s5 = -r13*(c1*c23*c4+s1*s4)-r23*(s1*c23*c4-c1*s4)+r33*(s23*c4);
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5_4 = atan2(s5,c5);
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*s23*s4;
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*c4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6_4 = atan2(s6,c6);

thetas = [theta11,theta2_1,theta31,theta4_1,theta5_1,theta6_1;...
    theta11,theta2_2,theta32,theta4_2,theta5_2,theta6_2;...
    theta12,theta2_3,theta31,theta4_3,theta5_3,theta6_3;...
    theta12,theta2_4,theta32,theta4_4,theta5_4,theta6_4;...
    theta11,theta2_1,theta31,theta4_1+pi,-theta5_1,theta6_1+pi;...
    theta11,theta2_2,theta32,theta4_2+pi,-theta5_2,theta6_2+pi;...
    theta12,theta2_3,theta31,theta4_3+pi,-theta5_3,theta6_3+pi;...
    theta12,theta2_4,theta32,theta4_4+pi,-theta5_4,theta6_4+pi];

end

