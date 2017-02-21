function [t, v0, v1, v2, v3, v4, v5] = joint_velocities(filename);
joints_pos = importdata(filename);

t = joints_pos(:, 2);
q0 = joints_pos(:, 3);
q1 = joints_pos(:, 4);
q2 = joints_pos(:, 5);
q3 = joints_pos(:, 6);
q4 = joints_pos(:, 7);
q5 = joints_pos(:, 8);

% Scaling factor
scale = 1; % 1.69;

% Extract velocities from joints and convert from degree/s to pixel/s
v0 = diff(q0).*scale;
v1 = diff(q1).*scale;
v2 = diff(q2).*scale;
v3 = diff(q3).*scale;
v4 = diff(q4).*scale;
v5 = diff(q5).*scale;
% v0 = (diff(q0)./diff(t)).*scale;
% v1 = (diff(q1)./diff(t)).*scale;
% v2 = (diff(q2)./diff(t)).*scale;
% v3 = (diff(q3)./diff(t)).*scale;
% v4 = (diff(q4)./diff(t)).*scale;
% v5 = (diff(q5)./diff(t)).*scale;
t = t(2:end);
 
