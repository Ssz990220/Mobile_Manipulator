function thetas= Inverse_Kinematics(robot, target_pose, params)
    T = robot.T_forward;
    eqn = T == target_pose;
    thetas = solve(eqn,params);
end
