%% Use this function def when not plotting curves
%function[curve, coeffs] = compute_coeffs(joint_pos)

%% Use this function def when Plotting curves
function[curve, coeffs, skel] = compute_coeffs(joint_pos)
[ee_cart_pos, j2_cart_pos] = compute_cart_pos(joint_pos);
skel = skeleton(j2_cart_pos, ee_cart_pos);
[curve, coeffs] = fit_curve(skel);
end