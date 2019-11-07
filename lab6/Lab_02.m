clc
clear
close all

% Cruise Control: System Modeling in the Frequency Domain
% 
% F = ma
% F = u - bv
% ma = u - bv 
% mv_dot + bv = u
% v_dot = (1/m) u - (b/m) v
% v_dot = B u + A v
%
% Since we are interested in controlling the speed of the vehicle, 
% the output equation is chosen as follows
%
% y = v 
% y = C v + D
    
% we are interested in controlling the speed of the vehicle
desiredVal = 10; % reference speed in m/s
m = 1000; % mass
b = 50;   % damping coefficient in N.s/m

%% compute the transfer function P(s)
P = tf(1,[m b])

%% introduce the PID controller
%% here we can right click on each gain and choose increment value and run section to see the effect
Ki = 15;   % integral constant
Kp = 300;   % proportional constant
Kd = 10;    % derivative constant
C = pid(Kp, Ki, Kd) % call the pid function

% apply a step function and compute the response
step(desiredVal*feedback(P*C,1))



 
 
 
