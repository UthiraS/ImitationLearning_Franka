%%File Paths
clear
file_path = "~/Pictures/adaptive_baseline_gazebo/initial estimation/trial";
trial_no = 4;

str_J = file_path+trial_no+"/modelerror.csv";
str_jvel1 = file_path+trial_no+"/j1vel.csv";
str_jvel2 = file_path+trial_no+"/j2vel.csv";

%% Read data
J = readtable(str_J,'NumHeaderLines',1);
J_vec = table2array(J);

jvel1 = readtable(str_jvel1,'NumHeaderLines',1);
jvel1_vec = table2array(jvel1);

jvel2 = readtable(str_jvel2,'NumHeaderLines',1);
jvel2_vec = table2array(jvel2);

%% Plot
figure(1)
hold on
subplot(3,1,1)
plot(J_vec,'LineWidth',1.5)
title('Model Error')
xlabel('Iteration#')
ylabel('J')

subplot(3,1,2)
plot(jvel1_vec,'LineWidth',1.5)
title("J1 input velocity")
xlabel('Iteration#')
ylabel('Velocity')

subplot(3,1,3)
plot(jvel2_vec,'LineWidth',1.5)
title("J2 input velocity")
xlabel('Iteration#')
ylabel('Velocity')
