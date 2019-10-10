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
%g = trapmf(x,[4 4 8 8]);
f = 1-g;
y = 1:15;
q = trapmf(y,[1 1 5 5]) + trapmf(y,[10 10 15 15]);
%q = trapmf(y,[5 5 10 10]);
p = 1-q;

for i=1:1931
    crispy_fusion_x(i) = f(floor(ref_pos(i,2)))*cam_pos(i,2)+g(floor(ref_pos(i,2)))*magic_pos(i,2);
    crispy_fusion_y(i) = p(floor(ref_pos(i,3)))*cam_pos(i,3)+q(floor(ref_pos(i,3)))*magic_pos(i,3);
end
subplot(2,1,1);
plot(crispy_fusion_x, crispy_fusion_y)
%g1 = fismf("trapmf", [[2 2 4 4],[6 6 8 8]]);
%g2 = fismf("trapmf", [8 8 10 10]);
%evalmf(g,5)
%evalmf(g,x)
%plot(x,evalmf(g1,x))
%plot(y,q)
%q(floor(max(cam_pos(:,3))))
%xlabel('trapmf, P = [1 5 7 8]')
%lim([-0.05 1.05])

g1 = trapmf(x,[1 1 2 3]) + trapmf(x,[8 9 10 10]);
%g = trapmf(x,[4 4 8 8]);
f1 = 1-g1;
q1 = trapmf(y,[1 1 2 5]) + trapmf(y,[10 12 15 15]);
%q = trapmf(y,[5 5 10 10]);
p1 = 1-q1;

for i=1:1931
    crispy_fusion_x1(i) = f1(floor(ref_pos(i,2)))*cam_pos(i,2)+g1(floor(ref_pos(i,2)))*magic_pos(i,2);
    crispy_fusion_y1(i) = p1(floor(ref_pos(i,3)))*cam_pos(i,3)+q1(floor(ref_pos(i,3)))*magic_pos(i,3);
end
subplot(2,1,2);
plot(crispy_fusion_x1, crispy_fusion_y1)

%% Task 4
% given variables
r = 0.25;
L = 0.8;

% new variables
theta = 0;
delta_theta = zeros(1,19309);
delta_D = zeros(1,19309);
delta_x = zeros(1,19309);
delta_y = zeros(1,19309);


% calculation of deltas
for i=1:19309
    delta_r = (enc_val(i+1,2)-enc_val(i,2));
    if (delta_r >= (pi()))
        delta_r = (2*pi()) - delta_r;
    end
    if (delta_r <= (-pi()))
        delta_r = (-2*pi()) - delta_r;
    end
    
    delta_l = (enc_val(i+1,3)-enc_val(i,3));
    if (delta_l >= (pi()))
        delta_l = (2*pi()) - delta_l;
    end
    if (delta_l <= (-pi()))
        delta_l = (-2*pi()) - delta_l;
    end
    
    delta_theta(i) = (delta_r-delta_l)*(r/L);
    delta_D(i) = (delta_r+delta_l)*(r/2);
    delta_x(i) = delta_D(i)*cos(theta);
    delta_y(i) = delta_D(i)*sin(theta);
    theta = theta + delta_theta(i);
end

%estimated positions
x_est = zeros(1,19310);
y_est = zeros(1,19310);
theta_est = zeros(1,19310);
x_est(1) = ref_pos(1,2);
y_est(1) = ref_pos(1,3);
theta_est(1) = 0;

%calculation of estimated positions
for i=1:19309
    x_est(i+1) = x_est(i)+delta_x(i);
    y_est(i+1) = y_est(i)+delta_y(i);
    theta_est(i+1) = theta_est(i)+delta_theta(i);
end

figure()
subplot(2,1,1);
plot(ref_pos(:,2), ref_pos(:,3));
title('Reference position');
subplot(2,1,2);
plot(x_est, y_est);
title('Estimated Position');

figure()
plot(1:1:19310, x_est);
figure()
plot(1:1:19310, theta_est);

%% Task 5a
updated_pos_x = zeros(1,19310);
updated_pos_y = zeros(1,19310);
updated_pos_x(1) = ref_pos(1,2);
updated_pos_y(1) = ref_pos(1,3);

for i=1:19309
    if (mod((i+1),10))
        updated_pos_x(i+1) = updated_pos_x(i)+delta_x(i);
        updated_pos_y(i+1) = updated_pos_y(i)+delta_y(i);
    else
        pos = floor((i+1)/10);
        updated_pos_x(i+1) = sensor_fusion(pos,2);
        updated_pos_y(i+1) = sensor_fusion(pos,3);
    end
end

figure()
subplot(2,1,1);
plot(ref_pos(:,2), ref_pos(:,3));
title('Reference position');
subplot(2,1,2);
plot(updated_pos_x, updated_pos_y);
title('Estimated Position with updates');

%% Task 5b
updated_pos_del_x = zeros(1,19310);
updated_pos_del_y = zeros(1,19310);
updated_pos_del_x(1) = ref_pos(1,2);
updated_pos_del_y(1) = ref_pos(1,3);

for i=1:19309
    if (mod((i+1),10))
        updated_pos_del_x(i+1) = updated_pos_del_x(i)+delta_x(i);
        updated_pos_del_y(i+1) = updated_pos_del_y(i)+delta_y(i);
    else
        pos = floor((i+1)/10);
        delaypos = i+1-4;
        updated_pos_del_x(delaypos) = sensor_fusion(pos,2);
        updated_pos_del_y(delaypos) = sensor_fusion(pos,3);
        updated_pos_del_x(delaypos+1) = updated_pos_del_x(delaypos)+delta_x(delaypos);
        updated_pos_del_y(delaypos+1) = updated_pos_del_y(delaypos)+delta_y(delaypos);
        updated_pos_del_x(delaypos+2) = updated_pos_del_x(delaypos+1)+delta_x(delaypos+1);
        updated_pos_del_y(delaypos+2) = updated_pos_del_y(delaypos+1)+delta_y(delaypos+1);
        updated_pos_del_x(delaypos+3) = updated_pos_del_x(delaypos+1)+delta_x(delaypos+2);
        updated_pos_del_y(delaypos+3) = updated_pos_del_y(delaypos+1)+delta_y(delaypos+2);
        updated_pos_del_x(delaypos+4) = updated_pos_del_x(delaypos+1)+delta_x(delaypos+3);
        updated_pos_del_y(delaypos+4) = updated_pos_del_y(delaypos+1)+delta_y(delaypos+3);
        
    end
end

figure()
subplot(2,1,1);
plot(ref_pos(:,2), ref_pos(:,3));
title('Reference position');
subplot(2,1,2);
plot(updated_pos_del_x, updated_pos_del_y);
title('Estimated Position with delayed updates');