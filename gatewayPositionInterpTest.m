
% Build the position interpoloant
rgwn = buildGatewayPosInterp();
% Call the interpolant
r = rgwn(0:10:1000000);
% Create plot
fig = figure; hold on; grid on
% Plot time and state
plot3(r(1, :), r(2, :), r(3, :), 'LineWidth', 2)
% Label axes
xlabel('$x$ (km)')
ylabel('$y$ (km)')
zlabel('$z$ (km)')
view(45, 30)
axis equal
view(90, 25)