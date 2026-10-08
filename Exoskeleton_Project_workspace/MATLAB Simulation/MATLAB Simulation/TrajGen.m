function [qd] = TrajGen( u )
%TRAJGEN Summary of this function goes here
%   Detailed explanation goes here

global t0 t1 t2 t3 t4 t5 t6

t0 = 0;
t1 = 5;
t2 = 5;

q1=0;
q2=0;

q1d=u(1);
q2d=u(2);
t=u(3);

%% regulation
T1=[1, t0, t0^2, t0^3, t0^4, t0^5;
    1, t1, t1^2, t1^3, t1^4, t1^5;
    0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
    0, 1, 2*t1, 3*t1^2, 4*t1^3, 5*t1^4;
    0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
    0, 0, 2, 6*t1, 12*t1^2, 20*t1^3;];

T2=[1, t0, t0^2, t0^3, t0^4, t0^5;
    1, t2, t2^2, t2^3, t2^4, t2^5;
    0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
    0, 1, 2*t2, 3*t2^2, 4*t2^3, 5*t2^4;
    0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
    0, 0, 2, 6*t2, 12*t2^2, 20*t2^3;];


aq1=(T1)\[q1; q1d; 0; 0; 0; 0];

aq2=(T2)\[q2; q2d; 0; 0; 0; 0];


% Generate Different trajectories at different times
p=[1; t; t^2; t^3; t^4; t^5]';
v=[0; 1; 2*t; 3*t^2; 4*t^3; 5*t^4]';

if t>=t1
    q1dp = 0;
    q2dp = 0;
else
    q1d=p*aq1;
    q1dp=v*aq1;

    q2d=p*aq2;
    q2dp=v*aq2; 
end

%% tracking
%q1d=q1d/2*sin(2*pi/t1*t)+q1d/2+q1;
%q2d=q2d/2*sin(2*pi/t2*t)+q2d/2+q2;
%q3d=q3d*sin(2*pi/t3*t)+q3;

%q1dp=q1d*pi/t1*cos(2*pi/t1*t);
%q2dp=q2d*pi/t2*cos(2*pi/t2*t);
%q3dp=q3d*2*pi/t3*sin(2*pi/t3*t);

qd=[q1d;q2d;q1dp;q2dp];
  
end

