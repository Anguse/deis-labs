clear all; close all;

X = load('Data.mat');
% T_cam = T_magic = 10T_enc = 10T_ref
% Positions and distance are described as relative units, radius of the
% robot is 1 unit. The resolution of all sources of information is 0.1 unit
enc_val = X.Enc;        %[timestamp, phi_r, phi_l]
ref_pos = X.RefPos;     %[timestamp, x, y]
cam_pos = X.CamPos;     %[timestamp, cx, cy]
magic_pos = X.MagicPos; %[timestmap, mx, my]

%% Task 1
figure(1);
subplot(3,1,1)
plot(ref_pos(:,2), ref_pos(:,3))
title('Reference position')
subplot(3,1,2)
plot(cam_pos(:,2), cam_pos(:,3))
title('Camera position')
subplot(3,1,3)
plot(magic_pos(:,2), magic_pos(:,3))
title('Magic position')
set(gcf,'Position',[1000 100 500 800])
% The camera position is curved far out on the edges since the camera is
% mounted above the robot with a wide-angle lens. The magic position is a
% good estimation close to the walls but poor further away from the walls

%% Task 2
A = .5
sensor_fusion = A*cam_pos+(1-A)*magic_pos;
% Compute the error residuals in relation to the reference position
r1 = norm(ref_pos(1:10:19310,2)-sensor_fusion(:,2)) + norm(ref_pos(1:10:19310,3)-sensor_fusion(:,3));
r2 = norm(ref_pos(1:10:19310,2)-cam_pos(:,2)) + norm(ref_pos(1:10:19310,3)-cam_pos(:,3));
r3 = norm(ref_pos(1:10:19310,2)-magic_pos(:,2)) + norm(ref_pos(1:10:19310,3)-magic_pos(:,3));
figure(2)
subplot(4,1,1)
plot(ref_pos(:,2), ref_pos(:,3))
title(['Reference position'])
subplot(4,1,2)
plot(cam_pos(:,2), cam_pos(:,3))
title(['Camera position, residual=' num2str(r2)])
subplot(4,1,3)
plot(magic_pos(:,2), magic_pos(:,3))
title(['Magic position, residual=' num2str(r3)])
subplot(4,1,4)
plot(sensor_fusion(:,2), sensor_fusion(:,3))
title(['Fused position, residual=' num2str(r1)])
set(gcf,'Position',[1000 100 500 800])

%% Task 3
figure(3)
x = 1:10;
g = trapmf(x,[1 1 4 4]) + trapmf(x,[8 8 10 10]);
f = 1-g;
y = 1:15;
q = trapmf(y,[1 1 5 5]) + trapmf(y,[10 10 15 15]);
p = 1-q;

for i=1:1931
    crispy_fusion_x(i) = f(floor(ref_pos(i,2)))*cam_pos(i,2)+g(floor(ref_pos(i,2)))*magic_pos(i,2);
    crispy_fusion_y(i) = p(floor(ref_pos(i,3)))*cam_pos(i,3)+q(floor(ref_pos(i,3)))*magic_pos(i,3);
end
plot(crispy_fusion_x, crispy_fusion_y)
%g1 = fismf("trapmf", [[2 2 4 4],[6 6 8 8]])
%g2 = fismf("trapmf", [8 8 10 10])


%evalmf(g,5)
%evalmf(g,x)
%plot(x,evalmf(g1,x))
%plot(y,q)
%q(floor(max(cam_pos(:,3))))
%xlabel('trapmf, P = [1 5 7 8]')
%lim([-0.05 1.05])



