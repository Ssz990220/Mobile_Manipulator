classdef Manipulability_layer
    
    properties
        pose;               %Ŀ����̬
        resolution;         %����Manipulability_map�ķֱ���
        edge_total;         %��ͼ�ܿ��
        map;                %�������
        manipulability_map; %�ɲ�����ͼ
    end
    
    methods
        function obj = Manipulability_layer(pose, resolution, edges_total)
             %�ɲ�����ͼ����ʽΪdouble���󣬴�СΪ3*��2*resolution+1��^2������Ϊx y �ɲ�����
            obj.pose = pose;                    %Ŀ����̬������̬Ϊdouble[4,4] �Ա���eef�Ŀռ�λ�ˣ�xyz roll pitch yaw��
            obj.resolution = resolution;        %��ͼ�ֱ���
            obj.edge_total = edges_total;       %��ͼ�ܳ�
            obj.map = cell(2*resolution+1);     %��ͼ�ɣ�2*resolution+1��^2���㹹�ɣ��õ�ͼΪcell��ʽ��cell��ÿ��Ԫ�ص�ֵΪһ������꣨��ʱ�Ƕ�άx��y��
            edge = edge_total/(2*resolution);  
            for i = 1:(2*resolution+1)
                for j = 1:(2*resolution + 1)
                    obj.map{i,j} = [(resolution+1-i)*edge (-resolution-1+j)*edge];   %������ͼmap���������Ϊ������base_coor��ԭ�㣨target�����������[x y]
                end
            end
            obj.manipulability_map = zeros(2*resolution+1);                         %��ʼ���ɲ�����ͼ
        end
        
        function obj = draw_map(obj, robot)                                         %���ƿɲ�����ͼ
            %   draw_map��obj, robot) robotΪ������
            %   ����˼·��
            %       ����map�е�ÿ���㣬��cell��Ӧλ�õ�ֵ����pose�е�����12��
            %       ����ikine���˶�ѧ������ؽڽǶ�
            %       ����ýǶ��¿ɲ�����
            %       ����obj.manipulability_map������
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

