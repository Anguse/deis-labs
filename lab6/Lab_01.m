clc
clear
close all
 


% Cruise Control: System Modeling
% 
% F = ma
% F = u - bv
% ma = u - bv 
% mv_dot + bv = u
% v_dot = (1/m) u - (b/m) v
%
% Since we are interested in controlling the speed of the vehicle, 
% the output equation is chosen as follows
%
% y = v

% we are interested in controlling the speed of the vehicle

tStart = 0; tEnd = 250; % time parameters
dt = 0.001; % sampling rate 
desiredVal = 10; % reference speed in m/s
m = 1000; % mass
b = 50;   % damping coefficient in N.s/m

emptyArray = zeros(tEnd/dt,1); % needed for the initialization of other arrays
y = emptyArray; % initialize the output speed  
t = emptyArray;  % initialize the time array
up = emptyArray;  % initialize the control
error = emptyArray; % keep track of the error
integral=0;   % initialize the integral sum
Ki = .5;   % integral constant
Kp = 50;  % proportional constant
Kd= 30;    % derivative constant

 
v = emptyArray; %Vehicle speed

% CONTROL LOOP 
for i=tStart+2:tEnd/dt   
    
    %% compute the PID control signal
    error(i)   =  desiredVal - y(i-1); % compute the error
    integral   =  integral+(error(i-1)+error(i))*dt/2;  % we use the trapezoid rule to approximate the integral 
    derivative =  (error(i)-error(i-1))/dt;  % compute the derivative of the error
      
    up(i) = Kp*error(i)+Ki*integral+Kd*derivative ;      % compute the PID control
    
    
    % compute the output signal    
    y(i) = y(i-1)+(1/m*up(i)-b/m*y(i))*dt;

    
    % log the time
    t(i) = t(i)+dt*i;
       
end

% Results
figure(1)
plot(t,y)
title ('Step Response')
xlabel('Time (seconds)')
ylabel('Amplitude')
axis([0 tEnd 0 12])
hold on
reference = ones(tEnd/dt,1)*desiredVal;
plot(t,reference,':')
 
