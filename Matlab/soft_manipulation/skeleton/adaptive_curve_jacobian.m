clear
clc

% Location and folder # to store plots
exp_no = "12b";
str_loc = "~/Pictures/adaptive_vs_spline/exp"+exp_no;
if not(isfolder(str_loc))
    mkdir(str_loc)
end

% Initialize start pose
start_joint_pos = [deg2rad(132), deg2rad(-22)];

% Initialize goal pose
goal_joint_pos = [deg2rad(144), deg2rad(38)];

% Estimation variables
window = 30; % size of sliding window
step = 0; % sin velocity resolution

% Goal curve & co-efficients
[goal_curve, goal_coeffs,goal_skel] = compute_coeffs(goal_joint_pos);

% Start curve & co-efficients
[curve,old_coeffs, skel] = compute_coeffs(start_joint_pos);

old_r = start_joint_pos; % Initializing old joint positions with current joint positions

ds =[]; % Change in curve co-effs
dr =[]; % Change in joint positions
qhat = zeros(4,2); %Shape Jacobian

gamma1 = 10e-5; % learning rate for initial estimation
gamma2 = 10e-8; % learning rate for online estimation

% Visual Servoing variables
lam = 5e-2*eye(4); % visual servoing gain
%lam = [1000 0 0 0; 0 5e-4 0 0; 0 0 5e-4 0; 0 0 0 5e-4];
rate =100;
r = rateControl(rate); % visual servoing sampling rate
thresh = 5e-1;
it = 1; % Iterator for frame vis

% Data recording variables
ds_ = []; % ee pos change vector
dr_ = []; % joint position change vector
Jplot = []; % model error vector for visualizing result
j_dot_ = []; % velocity vector for visualizing result
err_ = []; % error vector for visualizing result
traj_ = []; % storing robot trajectory for vis

% Video recording variables
robotvid = str_loc + "/robotVideo.avi";
curvevid = str_loc + "/curveVideo.avi";
robot_video = VideoWriter(robotvid);
robot_video.FrameRate = 25;
open(robot_video)

curve_video = VideoWriter(curvevid);
curve_video.FrameRate = 25;
open(curve_video)

% Visualization variables
[ee_start, j2_start] = compute_cart_pos(start_joint_pos); % Initializing old_pos with current ee position
[ee_goal, j2_goal] = compute_cart_pos(goal_joint_pos); % For visualizing robot in goal state

%% Estimating Initial Jacobian
% Move Robot locally to collect estimation data
while(it<=window)
    % generate sinusoidal vel
    cur_joint_pos = start_joint_pos + [sin(step) + 0.5*((rand - 0.5)), cos(step) + 0.3*((rand - 0.5))];
    %cur_joint_pos = start_joint_pos + [sin(step), sin(step)];
    
    % current robot state
    [curve, cur_coeffs,skel] = compute_coeffs(cur_joint_pos); % skeleton polynomial and polynomial co-efficients
    [ee_cart_pos, j2_cart_pos] = compute_cart_pos(cur_joint_pos); % joint states
    
    % compute change in state
    ds = [ds; shape_change(cur_coeffs, old_coeffs)];
    old_coeffs = cur_coeffs;
    
    dr = [dr; angle_change(cur_joint_pos, old_r)];
    old_r = cur_joint_pos;
    
    % Increase step
    step = step + 0.0017;
    
    % Data collection    
    ds_ = [ds_; ds(it,:)]; % storing change in ee pos for vis
    dr_ = [dr_; dr(it,:)]; % storing change in joint positions for vis
    traj_ = [traj_; ee_cart_pos]; % storing exploration trajectory
    
    % Plot robot state
    figure(1)
    plot([0, j2_cart_pos(1), ee_cart_pos(1)], [0, j2_cart_pos(2), ee_cart_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    %plot(curve,'b')
    plot(traj_(:,1),traj_(:,2), 'mo')
    %plot([0, j2_goal(1), ee_goal(1)], [0, j2_goal(2), ee_goal(2)], 'g'); % goal state
    plot(ee_goal(1),ee_goal(2),'g*', 'MarkerSize',12)
    goaltxt = sprintf('(%.2f, %.2f)',ee_goal(1),ee_goal(2));
    text(ee_goal(1) - 70, ee_goal(2) -20, goaltxt,'FontSize',10);
    vis_txt = 'Initial Shape Jacobian Estimation';
    text(-100,130,vis_txt)
    txt = append('Step#:',int2str(it));
    text(50,-65,txt)
    axis([-165 165 -165 165])
    title('Robot Trajectory')
    legend({'robot','trajectory','goal'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    %pause(0.1);
    frame = getframe(gcf);
    writeVideo(robot_video, frame)
    hold off
    
    %% Visualize Curve Fit
    figure(2)
    axis([-165 165 -165 165])
    hold on
    title('Polynomial Fit')
    pbaspect([1 1 1])
    grid on
%     plot(curve,'r-',skel(:,1),skel(:,2),'k.')
    fnplt(curve)
%     plot(goal_curve,'g',goal_skel(:,1),goal_skel(:,2),'g*')
    fnplt(goal_curve, 'g')
    legend({'current curve', 'goal curve'},'Location','northeastoutside')
    frame = getframe(gcf);
    writeVideo(curve_video, frame)
    
    hold off
    
    % Increase iterator
    it = it +1;
end

% Estimate Jacobian for local movements
count = 1;
while count<=window
    [J, qhat_dot] = compute_energy_functional(ds, dr, qhat, count, gamma1);
    qhat = qhat + qhat_dot;
    Jplot = [Jplot; J];
    count = count+1;
end

%% Error at start of VS
err = [thresh, thresh, thresh, thresh]; % Initializing error

%% Initial Jacobian
Q = qhat

%% Control Loop
disp('Entering Control Loop')
reset(r) % Resetting rate control object
while(abs(norm(err)) >= thresh)
    
    %% Compute error
    err = compute_coeffs_err(cur_joint_pos, goal_joint_pos);
    
    %% Error saturation
%     for i = 1:4
%         if(abs(err(i)) >= 0.000001)
%             err(i) = 1*(err(i)/abs(err(i)));
%         end
%     end
    
    %% Generate Velocity
    [j_dot, r_dot] = velocity_gen(Q, lam, err);
    
    %% Update robot state
    j_update = (1/rate)*j_dot;
    cur_joint_pos = cur_joint_pos + j_update;
    
    %% Compute updated curve coeffs
    [curve, cur_coeffs, skel] = compute_coeffs(cur_joint_pos);
    [ee_cart_pos, j2_cart_pos] = compute_cart_pos(cur_joint_pos);
    
    %% Compute change in state
    ds(1,:) = shape_change(cur_coeffs, old_coeffs); % replacing oldest data instance
    ds = transpose(circshift(transpose(ds),-1,2)); % left circular shift
    old_coeffs = cur_coeffs;

    dr(1,:) = angle_change(cur_joint_pos, old_r);
    dr = transpose(circshift(transpose(dr),-1,2)); % left circular shift
    old_r = cur_joint_pos;
    
    %% Update Jacobian
    [J, qhat_dot] = compute_energy_functional(ds, dr, Q, window, gamma2);
    Q = Q + qhat_dot;
    
    
    %% Data Collection
    ds_ = [ds_; ds(window,:)]; % storing change in ee pos for vis
    dr_ = [dr_; dr(window,:)]; % storing change in joint positions for vis
    traj_ = [traj_; ee_cart_pos];
    Jplot = [Jplot; J]; % Store model errors for vis
    j_dot_ = [j_dot_; j_dot]; % Store velocities for vis
    err_ = [err_; err]; % Store errors for vis
    
    %% Visualize robot state
    figure(1)
    plot([0, j2_cart_pos(1), ee_cart_pos(1)], [0, j2_cart_pos(2), ee_cart_pos(2)], 'r', 'LineWidth', 5); % current robot state
    hold on
    %quiver(ee_cart_pos(1), ee_cart_pos(2), r_dot(1), r_dot(2),100000, 'c')
    %plot(traj_(1:window,1),traj_(1:window,2), 'mo')
    plot(traj_(window:end,1),traj_(window:end,2), 'b--', 'LineWidth', 2)
    %plot([0, j2_goal(1), ee_goal(1)], [0, j2_goal(2), ee_goal(2)], 'g'); % goal state
    %plot(goal_curve,'g',goal_skel(:,1),goal_skel(:,2),'g')
    plot(ee_goal(1),ee_goal(2),'g*', 'MarkerSize',12)
    goaltxt = sprintf('(%.2f, %.2f)',ee_goal(1),ee_goal(2));
    text(ee_goal(1) - 70, ee_goal(2) -20, goaltxt,'FontSize',10);
    vis_txt = 'Servoing to Goal';
    text(-100,130,vis_txt)
    txt = append('Step#:',int2str(it));
    text(50,-65,txt)
    axis([-165 165 -165 165])
    title('Robot Trajectory')
    legend({'robot','trajectory','goal'},'Location','northeastoutside')
    pbaspect([1 1 1])
    grid on
    %pause(0.1);
    frame = getframe(gcf);
    writeVideo(robot_video, frame)
    hold off
    
    %% Visualize Curve Fit
    figure(2)
    axis([-165 165 -165 165])
    hold on
    title('Polynomial Fit')
    pbaspect([1 1 1])
    grid on
%     plot(curve,'r-',skel(:,1),skel(:,2),'k.')
    fnplt(curve)
%     plot(goal_curve,'g',goal_skel(:,1),goal_skel(:,2),'g*')
    fnplt(goal_curve, 'g')
    legend({'current curve', 'goal curve'},'Location','northeastoutside')
    frame = getframe(gcf);
    writeVideo(curve_video, frame)
    
    hold off
    
    %% Increment iterator
    it = it+1;
    hold off
    %% Rate control
%     r.TotalElapsedTime 0.02s
    waitfor(r);
end
close(robot_video)
close(curve_video)
%% Plot Results
disp('Generating results')

figure()
plot(Jplot)
title('Model Error')
xlabel('Steps')
ylabel('Estimation Error')
model_err = str_loc+"/model_err.png";
saveas(gcf, model_err)

figure()
subplot(2,1,1)
plot(j_dot_(:,1))
title('Velocity J1')
xlabel('steps')
ylabel('Velocity')

subplot(2,1,2)
plot(j_dot_(:,2))
title('Velocity J2')
xlabel('steps')
ylabel('Velocity')
vel = str_loc+"/vel.png";
saveas(gcf, vel)

figure()
subplot(4,1,1)
plot(err_(:,1))
title('coefficient 1')
xlabel('steps')
ylabel('error')

subplot(4,1,2)
plot(err_(:,2))
title('coefficient 2')
xlabel('steps')
ylabel('error')

subplot(4,1,3)
plot(err_(:,3))
title('coefficient 3')
xlabel('steps')
ylabel('error')

subplot(4,1,4)
plot(err_(:,4))
title('coefficient 4')
xlabel('steps')
ylabel('error')
err = str_loc+"/coeff_err.png";
saveas(gcf, err)
% 
% subplot(4,1,4)
% plot(err_(:,4))
% title('coefficient 4')
% xlabel('steps')
% ylabel('error')

%% Error Norm compute & plot
for i = 1:length(err_)
    err_norm(i) = norm(err_(i,:));
end
figure()
plot(err_norm)
title('Coefficient Error Magnitude')
legend('coefficient error magnitude')
xlabel('steps')
ylabel('error magnitude')
err_normloc = str_loc+"/coeff_err_norm.png";
saveas(gcf,err_normloc)

%% Save Trajectory Plot
figure(1)
traj = str_loc+"/traj.png";
saveas(gcf, traj)

%% Video Processing
video_processing(str_loc);