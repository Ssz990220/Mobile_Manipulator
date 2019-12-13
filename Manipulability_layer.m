classdef Manipulability_layer
    
    properties
        pose;               %目标姿态
        resolution;         %绘制Manipulability_map的分辨率
        edge_total;         %地图总宽度
        map;                %方格点阵
        manipulability_map; %可操作度图
    end
    
    methods
        function obj = Manipulability_layer(pose, resolution, edges_total)
             %可操作都图，格式为double矩阵，大小为3*（2*resolution+1）^2，各列为x y 可操作度
            obj.pose = pose;                    %目标姿态，该姿态为double[4,4] 以表征eef的空间位姿（xyz roll pitch yaw）
            obj.resolution = resolution;        %地图分辨率
            obj.edge_total = edges_total;       %地图总长
            obj.map = cell(2*resolution+1);     %地图由（2*resolution+1）^2个点构成，该地图为cell格式，cell中每个元素的值为一点的坐标（暂时是二维x，y）
            edge = edge_total/(2*resolution);  
            for i = 1:(2*resolution+1)
                for j = 1:(2*resolution + 1)
                    obj.map{i,j} = [(resolution+1-i)*edge (-resolution-1+j)*edge];   %构建地图map，点的坐标为机器人base_coor下原点（target）的相对坐标[x y]
                end
            end
            obj.manipulability_map = zeros(2*resolution+1);                         %初始化可操作都图
        end
        
        function obj = draw_map(obj, robot)                                         %绘制可操作度图
            %   draw_map（obj, robot) robot为机器人
            %   程序思路：
            %       遍历map中的每个点，将cell对应位置的值赋给pose中第四列12行
            %       利用ikine逆运动学求出各关节角度
            %       计算该角度下可操作度
            %       生成obj.manipulability_map的新行
            %       [x,y,manipulability]
            for i = 1:(2*obj.resolution+1)
                for j = 1:(2*obj.resolution+1)
                    pose =obj.pose;
                    x = obj.map{i,j}(1);
                    y = obj.map{i,j}(2);
                    pose(1,4)=x;
                    pose(2,4)=y;
                    thetas = robot.ikine(pose);
                    manipulability = robot.cal_manipulability(thetas);
                    obj.manipulability_map((i-1)*(2*obj.resolution+1),:) = [x y manipulability];
                end
            end
            
        end
    end
end

