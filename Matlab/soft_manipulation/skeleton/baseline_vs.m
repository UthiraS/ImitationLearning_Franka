clear
clc

% Initialize start pose
cur_joint_pos = [deg2rad(138), deg2rad(-45)];

% Initialize goal pose
goal_joint_pos = [deg2rad(35), deg2rad(-95)];

% Visual Servoing variables
lam = [3, 0; 0 ,3]; % visual servoing gain
rate =100; % control loop frequency
r = rateControl(rate); % visual servoing sampling rate
thresh = 10e-1;
err = thresh;

% Visualization stuff
[ee_goal, j2_goal] = compute_cart_pos(goal_joint_pos); % For visualizing robot in goal state
it =1; %iterator

% Data recording variables
j_dot_ = []; % velocity vector for visualizing result
err_ = []; % error vector for visualizing result
traj_ = []; % storing robot trajectory for vis
th1_ = []; % storing joint1 angles
th2_ = []; % storing joint2 angles
coeffs_plot_ = []; % storing curve co-efficients
norm_coeffs_plot_ = []; % normalizing coeffs and storing

%% Visual Servoing Control Loop
while(abs(norm(err)) >= thresh)
    %% Compute error
    err = compute_err(cur_joint_pos, goal_joint_pos);
    
    %% Compute end effector Position
    [ee_pos, j2_pos] = compute_cart_pos(cur_joint_pos);
    
    %% Compute Curve Coefficients
    [curve, coeffs,skel] = compute_coeffs(cur_joint_pos);
    coeffs_plot_ = [coeffs_plot_; coeffs];
    norm_coeffs_plot_ = [norm_coeffs_plot_; coeffs/norm(coeffs)];
    
    %% Update Robot Jacobian
    Q = get_real_jacobian(cur_joint_pos);
    % det(Q)
    
    %% Generate Velocity
    [j_dot, r_dot] = velocity_gen(Q, lam, err);
    
    %% Update robot state
    j_update = (1/rate)*j_dot; 
    cur_joint_pos = cur_joint_pos + j_update;
    
    %% Data Collection
    traj_ = [traj_; ee_pos];
    j_dot_ = [j_dot_; j_dot]; % Store velocities for vis
    err_ = [err_; err]; % Store errors for vis
    th1_ = [th1_;cur_joint_pos(1)];
    th2_ = [th2_;cur_joint_pos(2)];
    
    %% Visualize robot state
    figure(1)
    plot([0, j2_pos(1), ee_pos(1)], [0, j2_pos(2), ee_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    quiver(ee_pos(1), ee_pos(2), r_dot(1), r_dot(2), 'm')
    plot(traj_(:,1),traj_(:,2), 'b--', 'LineWidth', 2)
    %plot([0, j2_goal(1), ee_goal(1)], [0, j2_goal(2), ee_goal(2)], 'g'); % goal state
    plot(ee_goal(1),ee_goal(2),'g*', 'MarkerSize',12)
    txt = append('Step#:',int2str(it));
    text(50,-45,txt)
    axis([-165 165 -165 165])
    title('Robot Trajectory')
    legend({'robot','cartesian velocity','trajectory','goal'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    %legend('boxoff')
    %pause(1);
    hold off
    
    %% Visualize Curve
    figure(2)
    %plot([0, j2_pos(1), ee_pos(1)], [0, j2_pos(2), ee_pos(2)]); % current robot state
    hold on
    fnplt(curve)
%     plot(curve,skel(:,1),skel(:,2))
    axis([-165 165 -165 165])
    title('Estimated Skeleton')
    %legend({'current robot pose', 'skeleton'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    hold off
    %% Increment iterator
    it = it+1;
    
    %% Rate control
%     r.TotalElapsedTime
    waitfor(r);
end

%% Plot Results
disp('Generating results')

figure()
plot(rad2deg(th1_))
hold on
plot(rad2deg(th2_))
title('Angle positions')
xlabel('time')
ylabel('theta')
legend('th1', 'th2')

figure()
subplot(2,1,1)
plot(j_dot_(:,1))
title('Velocity J1')
xlabel('time')
ylabel('Velocity')

subplot(2,1,2)
plot(j_dot_(:,2))
title('Velocity J2')
xlabel('time')
ylabel('Velocity')

figure()
plot(err_(:,1), 'b')
hold on
plot(err_(:,2), 'g')
legend('ErrorX', 'ErrorY')
title('Pose Error')
xlabel('Steps')
ylabel('Estimation Error')

figure()
plot(coeffs_plot_(:,1))
hold on
plot(coeffs_plot_(:,2))
plot(coeffs_plot_(:,3))
plot(coeffs_plot_(:,4))
% plot(coeffs_plot_(:,5))
legend('a','b','c','d','e')
title('Curve Co-efficients')
xlabel('Steps')
ylabel('Co-efficient')

figure()
plot(norm_coeffs_plot_(:,1))
hold on
plot(norm_coeffs_plot_(:,2))
plot(norm_coeffs_plot_(:,3))
plot(norm_coeffs_plot_(:,4))
% plot(norm_coeffs_plot_(:,5))
legend('a','b','c','d','e')
title('Normalized Curve Co-efficients')
xlabel('Steps')
ylabel('Normalized Co-efficient')