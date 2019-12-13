function manipulability = cal_manipulability(robot, params)
    px = robot.T_forward(1,4);
    py = robot.T_forward(2,4);
    pz = robot.T_forward(3,4);
    J = jacobian([px,py,pz],params);
    manipulability = J;
end

