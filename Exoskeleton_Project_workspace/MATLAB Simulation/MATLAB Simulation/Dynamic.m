function Qpp = Dynamic(u)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
% m1 = 15;
% m2 = 5;
% q1 = 0;
% q2 = 0;
% 
% dq1 = 0;
% dq2 = 0;
% 
% Q = [q1; q2];
% Qp = [dq1; dq2];

% L1 = 0.32;
% L2 = 0.32;
% L3 = 0.01;

q1 = u(1);
q2 = u(2);

dq1 = u(3);
dq2 = u(4);

Q = [q1; q2];
Qp = [dq1; dq2];

L1 = u(5);
L2 = u(6);
L3 = u(7);

m1 = u(8);
m2 = u(9); 

t = u(10);

gx = u(11);
gy = u(12);
gz = u(13);

Beta=diag([u(14); u(15)]);

Tao = [u(16); u(17)];

I111 = 0.4;
I112 = 0.2;
I113 = 0.2;
I122 = 0.4;
I123 = 0.2;
I133 = 0.4;

I211 = 0.4;
I212 = 0.2;
I213 = 0.2;
I222 = 0.4;
I223 = 0.2;
I233 = 0.4;

M = [I133 + (L2^2*m2)/4 + I222*cos(q2)^2 + I211*sin(q2)^2 + 2*I212*cos(q2)*sin(q2), - I223*cos(q2) - I213*sin(q2); - I223*cos(q2) - I213*sin(q2), I233];
C = [dq2*(I212*cos(2*q2) + (I211*sin(2*q2))/2 - (I222*sin(2*q2))/2), dq1*(I212*cos(q2)^2 - I212*sin(q2)^2 + I211*cos(q2)*sin(q2) - I222*cos(q2)*sin(q2)) - dq2*(I213*cos(q2) - I223*sin(q2)); -dq1*(I212*cos(2*q2) + (I211*sin(2*q2))/2 - (I222*sin(2*q2))/2), 0];
G=  [-(L2*m2*(gx*cos(q1) + gy*sin(q1)))/2; 0];

% compensate G in Dynamic instead in Tau(MATLAB Version)
Tao = Tao;

Qpp = (M)\(Tao-C*Qp-G-Beta*Qp);

end