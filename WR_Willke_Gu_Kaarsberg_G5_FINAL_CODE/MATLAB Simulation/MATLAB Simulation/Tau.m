function [ tau ] = Tau( u )
%TAU Summary of this function goes here
%   Detailed explanation goes here
% Times are defined in TrajGen.m file
global t0 t1 t2 t3 t4 t5 t6 Q_int

t=u(1);
t_Step = str2num(get_param('DSimulator_R2023a','FixedStep'));
if t== 0
    Q_int = zeros(2,1);
end

q1d=u(2);
q2d=u(3);
Qd=[q1d;q2d];

q1dp=u(4);
q2dp=u(5);
Qdp=[q1dp;q2dp];

Kd=diag([u(6);u(7)]);
Kp=diag([u(8);u(9)]);              
Ki=diag([u(10);u(11)]);

q1=u(12);
q2=u(13);
Q=[q1;q2];

q1p=u(14);
q2p=u(15);
Qp=[q1p;q2p];

%Joint Errors
DeltaQ=Q-Qd;

Q_int = Q_int+DeltaQ*t_Step;
DeltaQp=Qp-Qdp;

%Robot Parameters
m1=u(16);
m2=u(17);

gx=u(18);
gy=u(19);
gz=u(20);

L1=u(21);
L2=u(22);
L3=u(23); 

%TEST
tau1=u(24);
tau2=u(25);
tau = [tau1; tau2];

%G=[(L2*gx*m2*cos(q1))/2 + (L2*gy*m2*sin(q1))/2; 0];
G=[-(L2*m2*(gx*cos(q1) + gy*sin(q1)))/2; 0];
%Define controllers PD, PD+G, PID
% controller for PD
%tauc = -Kp * DeltaQ - Kd * DeltaQp;

% controller for PD+G
tauc = -Kp * DeltaQ - Kd * DeltaQp + G;

% controller for PID
%tauc = -Kp * DeltaQ - Kd * DeltaQp - Ki * Q_int + G;

% controller for force, G compensation in Dynamic
%tauc = -Kp * tau;

tau=[tauc;DeltaQ;DeltaQp];
 
end

