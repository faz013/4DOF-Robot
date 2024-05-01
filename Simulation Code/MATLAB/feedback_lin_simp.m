clear all
syms q2 q3 q4 q_dot2 q_dot3 q_dot4 q_ddot2 q_ddot3 q_ddot4 g 
syms k1 k2 k3 k4 k5 k6
syms qd2 qd3 qd4 
syms qd_dot2 qd_dot3 qd_dot4 qd_ddot2 qd_ddot3 qd_ddot4

digitsOld = digits(3);
g = 9.81;

m1 = 0.230;
m2 = 0.230;
l1 = 0.14;
l2 = 0.13;

% M = [0.0126 + 0.00166*cos(q3+q4) + 0.00643*cos(q3),0.00166*cos(q3+q4)+0.00643*cos(q3),0.00166*cos(q3+q4);...
%     0.00643+0.00643*cos(q3+q4)+0.00166*cos(q4),0.00643+0.00166*cos(q4),0.00166*cos(q4);...
%     0.00903+0.00166*cos(q3+q4)+0.00166*cos(q4),0.00903+0.00166*cos(q4),0.00903];
M = [l2*l2*m2+2*l1*l2*cos(q3)+l1*l1*(m1+m2),l2*l2*m2+l1*l2+m2*cos(q3);l2*l2*m2+l1*l2+m2*cos(q3),l2*l2*m2];

C = [-m2*l1*l2*sin(q3)*q_dot3^2-2*m2*l1*l2*sin(q3)*q_dot2*q_dot3;...
    m2*l1*l2*sin(q3)*q_dot2^2];

G = [m2*l2*g*cos(q2+q3)+(m1+m2)*l1*g*cos(q2);m2*l2*g*cos(q2+q3)];

q_ddot = [q_ddot2; q_ddot3];%; q_ddot4];
qd_ddot = [qd_ddot2;qd_ddot3];%;qd_ddot4];

q_dot = [q_dot2; q_dot3];% ;q_dot4];
qd_dot = [qd_dot2;qd_dot3]; %qd_dot4];

q = [q2;q3]%;q4];
qd = [qd2;qd3]%;qd4];
% C = [- 0.00643*q_dot2^2*sin(q3) - 0.00643*q_dot3^2*sin(q3) - 0.00166*q_dot2^2*sin(q3 + q4) - 0.00166*q_dot3^2*sin(q3 + q4) - 0.00166*q_dot4^2*sin(q3 + q4) - 0.0129*q_dot2*q_dot3*sin(q3) - 0.00333*q_dot2*q_dot3*sin(q3 + q4) - 0.00333*q_dot2*q_dot4*sin(q3 + q4) - 0.00333*q_dot3*q_dot4*sin(q3 + q4);...
%     + 0.00643*q_dot2^2*sin(q3) - 0.00166*q_dot2^2*sin(q4) - 0.00166*q_dot3^2*sin(q4) - 0.00166*q_dot4^2*sin(q4) - 0.00333*q_dot2*q_dot3*sin(q4) - 0.00333*q_dot2*q_dot4*sin(q4) - 0.00333*q_dot3*q_dot4*sin(q4);...
%     + 0.00166*q_dot2^2*sin(q4) + 0.00166*q_dot3^2*sin(q4)  + 0.00166*q_dot2^2*sin(q3 + q4) + 0.00333*q_dot2*q_dot3*sin(q4);...
% ];
% 
% G = [+ 1.75*cos(q3 + q4)+ 0.0717*g*cos(q2);1.75*cos(q4) + 0.0368*g*cos(q2 + q3);+ 9.5 + 0.0095*g*cos(q2 + q3 + q4)];

%Kv = [k1 0 0;0 k2 0;0 0 k3];

%Kp =[k4 0 0;0 k5 0;0 0 k6];
Kp = [k3 0;0 k4];
Kv = [k1 0;0 k2];


%E_dot = qd_dot-q_dot;
%E = qd-q;
syms e_dot2 e_dot1 e1 e2
e = [e1; e2];
e_dot = [e_dot1; e_dot2 ];
u = M*(qd_ddot + Kv*e_dot + Kp*e) %+C +G;

u = vpa(u);

simplify(u)