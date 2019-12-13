classdef robot < handle
    %ROBOT 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        Link_bin;
        Link_count;
        z;
        T_forward;
    end
    
    methods
        function obj = robot(z)
            obj.Link_bin = {};
            obj.Link_count = 0;
            obj.z = z;
        end
        
        function obj = Add_link(obj,link)
            if obj.Link_count ==0
                obj.Link_bin{obj.Link_count+1} = link;
                obj.T_forward = link.T;
                obj.Link_count = obj.Link_count + 1;
            else
            obj.Link_bin{obj.Link_count+1} = link;
            obj.Link_count = obj.Link_count + 1;
            obj.T_forward = obj.T_forward * link.T;
            end
        end
        
%         function T_eff = fkine(thetas,obj)
%             param = [];
%             for i = 1:obj.Link_count
%                 param(i)=obj.Link_bin[i]
%            end
        
%         function target_thetas = ikine(target_pose, obj)
%            target_thetas = solve(obj.fkine() = target_pose); 
%         end
        
%         function manipulability = cal_manipulability(thetas,obj)
%             px = obj.T_forward(1,4);
%             py = obj.T_forward(2,4);
%             pz = obj.T_forward(3,4);
%             J = jacobian([px,py,pz],[])
%         end
    end
end

