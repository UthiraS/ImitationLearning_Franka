clear;

l1 = 90; 
l2 = 75;

th1 = -1.5871;
th2 = 1.3658;
% syms th1, th2;

eeX = l1*cos(th1) + l2*cos(th1 + th2);
eeY = l1*sin(th1) + l2*sin(th1 + th2);

j2X = l1*cos(th1);
j2Y = l1*sin(th1);

baseX = 0;
baseY = 0;

%% Equation of Link1
link1PtsX = linspace(0, j2X, 10);
link1PtsY = linspace(0, j2Y, 10);

%% Equation of Link2
link2PtsX = linspace(j2X, eeX, 10);
link2PtsY = linspace(j2Y, eeY, 10);

%% Plot robot
figure()
plot([0, j2X, eeX], [0, j2Y, eeY], 'c', 'LineWidth', 5);
hold on
plot(link1PtsX(:), link1PtsY(:), 'r*', 'LineWidth',2)
plot(link2PtsX(:), link2PtsY(:), 'r*', 'LineWidth',2)
%axis([-165 165 -165 165])
%grid on
%pbaspect([1 1 1])

%% Fit a 2nd Degree Polynomial
curvePtsX = [link1PtsX, link2PtsX(2:end)]
curvePtsY = [link1PtsY, link2PtsY(2:end)]
xpts = [0, j2X, eeX];
ypts = [0, j2Y, eeY];
% p = polyfit(curvePtsX,curvePtsY,6);
% x1 = linspace(0,eeX);
% f1 = polyval(p,x1);
f=fit(transpose(curvePtsX),transpose(curvePtsY),'spline')
cs = spap2(2,2,transpose(curvePtsX), transpose(curvePtsY))
% plot(x1, f1, 'm--')
%plot(f,'r-', curvePtsX, curvePtsY)
fnplt(cs,'b')
