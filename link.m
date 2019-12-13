classdef link
    
    properties
        T
        theta
    end
    
    methods
        function obj = link(alpha_i_1, a_i_1, d_i, theta_i)
            %LINKS 构造此类的实例
            %   此处显示详细说明
            obj.T = [cos(theta_i),-sin(theta_i),0,a_i_1;...
                sin(theta_i)*cos(alpha_i_1),cos(theta_i)*cos(alpha_i_1),-sin(alpha_i_1),-sin(alpha_i_1)*d_i;...
                sin(theta_i)*sin(alpha_i_1),cos(theta_i)*sin(alpha_i_1),cos(alpha_i_1),cos(alpha_i_1)*d_i;...
                0,0,0,1];
            obj.theta = theta_i;
        end
        
    end
end
