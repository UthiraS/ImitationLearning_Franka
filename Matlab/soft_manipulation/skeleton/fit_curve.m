function [f, coeffs] = fit_curve(skeletonPts) %#codegen

curvePtsX = skeletonPts(:,1);
curvePtsY = skeletonPts(:,2);

%% Fitting polynomial to skeleton
% f=fit(curvePtsX,curvePtsY,'poly2');
%  coeffs = [];   
% curve co-efficients
% coeffs = [f.p1, f.p2, f.p3];
% coeffs = [f.p1, f.p2, f.p3, f.p4, f.p5];
%coeffs = [f.p1, f.p2];

%% Fitting spline to skeleton
%f = spap2(2,3,curvePtsX,curvePtsY); % 2 quadratic segments
f = spap2(2,2,curvePtsX,curvePtsY); % 2 linear segments
sp = spcrv(skeletonPts.',2); % spline curve order 2
figure(2)
plot(sp(1,:),sp(2,:),'k','LineWidth',4);
hold on
cscvn(skeletonPts.')
fnplt(cscvn(skeletonPts.'), 'm',3);

%f1 = spap2(newknt(f),2,curvePtsX, curvePtsY)
coeffs = f.coefs;
end