%Calculates the Jacobian matrix for our 4DOF robotic manipulator
% a is theta_1, b is theta_2, c is theta_3, d is theta_4 and _v represents
% the angular velocity of the joint, and _a the angular acceleration
% gamma refers to the angle of EE about the x-axis of the base frame
% beta refers to the angle of EE about the y-axis of the base frame
% alpha refers to the angle of EE about the z-axis of the base frame
clear all
digitsOld = digits(3);
syms a(t) b(t) c(t) d(t) a_v b_v c_v d_v a_a b_a c_a d_a real
syms t1 t2 t3 t4
a_v = diff(a,t);
b_v = diff(b,t);
c_v = diff(c,t);
d_v = diff(d,t);

% a = 1.5;
% b = 1.5;
% c = 1.5;
% d = 1.5;


l1 = 70/1000;
l2 = 125/1000;
l3 = 125/1000;
l4 = 50/1000;

m1 = 0.03 + 0.2; 
m2 = 0.064 + 0.2;
m3 = 0.064 + 0.2;
m4 = 0.03;

%WILL NEED TO CONSIDER OUT OF PLANE INERTIAS AS LINKS ARE NOT SYMMETRIC

Ixx1 = 169.141/(100^2); 
Iyy1 = 166.040/(100^2);
Izz1 = 140.187/(100^2);
Ixy1 = -11.427/(100^2) ;
Ixz1 = 60.169/(100^2);
Iyz1 = 14.503/(100^2);

Ixx2 = 514.332/(100^2);
Iyy2 = 300.100/(100^2);
Izz2 = 490.333/(100^2);
Ixy2 = -95.070/(100^2);
Ixz2 = 96.881/(100^2);
Iyz2 = -24.355/(100^2);

Ixx3 = 514.332/(100^2);
Iyy3 = 300.100/(100^2);
Izz3 = 490.333/(100^2);
Ixy3 = -95.070/(100^2);
Ixz3 = 96.881/(100^2);
Iyz3 = -24.355/(100^2);

Ixx4 = 65.155/(100^2);
Iyy4 = 84.968/(100^2);
Izz4 = 32.347/(100^2);
Ixy4 = -6.71/(100^2);
Ixz4 = 2.929/(100^2);
Iyz4 = -0.741/(100^2);

dx = cos(a)*(l2*cos(b) + l3*cos(b+c)) + l4*cos(a)*cos(b+c+d);
dy = sin(a)*(l2*cos(b) + l3*cos(b+c)) + l4*sin(a)*cos(b+c+d);
dz = l1 + l2*sin(b) + l3*sin(b+c) + l4*sin(b+c+d);
gamma = pi/2;
beta = b + c + d;
alpha = a;

q_vel = [a_v, b_v, c_v, d_v]';

q_acc = [a_a, b_a, c_a, d_a]';

J = simplify(jacobian([dx, dy, dz, gamma, beta, alpha], [a, b, c, d]));

%J = simplify([diff(dx,a) diff(dx,b) diff(dx,c) diff(dx, d); ...
%    diff(dy,a) diff(dy,b) diff(dy,c) diff(dy, d); ...
%    diff(dz,a) diff(dz,b) diff(dz,c) diff(dz, d); ...
%    diff(gamma,a) diff(gamma,b) diff(gamma,c) diff(gamma, d); ...
%    diff(beta,a) diff(beta,b) diff(beta,c) diff(beta, d); ...
%    diff(alpha,a) diff(alpha,b) diff(alpha,c) diff(alpha, d)]);

%Calculates the Jacobian corresponding to each individual link
%L_x/2 can be replaced by r_x if we disregard the assumption that the
%centre of mass of each link lies at the midpoint, we will probably have
%to change this due to significant weight of motors at joints

J_1 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 1 0 0 0];
J_2 = [0 0 0 0; (l2/2)*cos(b) 0 0 0; 0 l2/2 0 0; sin(b) 0 0 0; 0 -1 0 0; cos(b) 0 0 0];
J_3 = [0 (l2/2)*sin(c) 0 0; (l3/2)*cos(b+c) + l2*cos(b) 0 0 0; 0 l3/2 + l2*cos(c) l3/2 0; sin(b+c) 0 0 0; 0 -1 -1 0; cos(b+c) 0 0 0];
J_4 = [0 l2*sin(b+c+d) + l3*sin(dz) l3*sin(d) 0; l3*cos(b+c) + l2*cos(b) + (l4/2)*cos(b+c+d) 0 0 0; 0 l4/2 + l2*cos(c+d) + l3*cos(d) l4/2 + l3*cos(d) l4/2; sin(b+c+d) 0 0 0; 0 -1 -1 -1; cos(b+c+d) 0 0 0];

%The inertia matrix being diagional i.e. only Ix, Iy, and Iz assumes that
%link is perfectly symmetric about the xy and xz plane

%This matrix is true only if we attach the coordinate fram at the mass
%center of each link

M_1 = [m1 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 m1 0 0 0; 0 0 0 Ixx1 Ixy1 Ixz1; 0 0 0 Ixy1 Iyy1 Iyz1; 0 0 0 Ixz1 Iyz1 Izz1];
M_2 = [m1 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 m2 0 0 0; 0 0 0 Ixx2 Ixy2 Ixz2; 0 0 0 Ixy2 Iyy2 Iyz2; 0 0 0 Ixz2 Iyz2 Izz2];
M_3 = [m1 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 m3 0 0 0; 0 0 0 Ixx3 Ixy3 Ixz3; 0 0 0 Ixy3 Iyy3 Iyz3; 0 0 0 Ixz3 Iyz3 Izz3];
M_4 = [m1 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 m4 0 0 0; 0 0 0 Ixx4 Ixy4 Ixz4; 0 0 0 Ixy4 Iyy4 Iyz4; 0 0 0 Ixz4 Iyz4 Izz4];

%D is the manipulator inertia matrix

D = transpose(J_1)*M_1*J_1 + transpose(J_2)*M_2*J_2 + transpose(J_3)*M_3*J_3 + transpose(J_4)*M_4*J_4;

%Calculates the Christoffel symbols for the manipulator

q=[a, b, c, d];
ch=sym(zeros(4,4,4));

for i=1:4
    for j=1:4
        for k=1:4

            ch(i,j,k) = 0.5*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)));
            
        end 
    end
end

%Uses the Christoffel symbols calculated to form the Coriolis matrix for
%the manipulator

C1 = sym(zeros(4,1));

for j=1:4

C1(j,1) = ch(1,1,j)*a_v + ch(1,2,j)*b_v + ch(1,3,j)*c_v + ch(1,4,j)*d_v;
         
end

C2 = sym(zeros(4,1));

for j=1:4

C2(j,1) = ch(2,1,j)*a_v + ch(2,2,j)*b_v + ch(2,3,j)*c_v + ch(2,4,j)*d_v;
         
end

C3 = sym(zeros(4,1));

for j=1:4

C3(j,1) = ch(3,1,j)*a_v + ch(3,2,j)*b_v + ch(3,3,j)*c_v + ch(3,4,j)*d_v;
         
end

C4 = sym(zeros(4,1));

for j=1:4

C4(j,1) = ch(4,1,j)*a_v + ch(4,2,j)*b_v + ch(4,3,j)*c_v + ch(4,4,j)*d_v;
         
end
    
C = simplify([C1 C2 C3 C4]);   

C_trial=sym(zeros(4,4,4));

for i=1:4
    for j=1:4
        for k=1:4

C_trial = sum(ch(k,i,j)*q_vel(i)*q_vel(j));

        end
    end
end

%Calculates the gravity forces on the robotic manipulator

%h_i is the height of the mass centre of the ith link
%Again the l_1/2 can be replaced with the calculated value of the centre
%of mass of each link 

h_1 = l1/2;
h_2 = l1 + (l2/2)*sin(b);
h_3 = l1 + l2*sin(b) + (l3/2)*sin(b+c);
h_4 = l1 + l2*sin(b) + l3*sin(b+c) + (l4/2)*(b+c+d);

%Potential energy of system

g=9.81;

V = g*(m1*h_1 + m2*h_2 + m3*h_3 + m4*h_4);

%Components of gravity forces

G = simplify(sym(zeros(4,1)));

for j=1:4
    
    G(j,1) = diff(V,q(j));
    
end
state_terms = inv(D)*(C*q_vel+G);

for x = 1:4
state_model(x,1) = subs(state_terms(x,1),[a b c d a_v b_v c_v d_v g],[x1 x2 x3 x4 diff(x1,t) diff(x2,t) diff(x3,t) diff(x4,t) 9.81]);
end

% D_lin = sym(zeros(4));
% C_lin = sym(zeros(4));
% G_lin = sym(zeros(4,1));
% 
% x= 1;
% y = 1;
% % next to the q1 q2 q3 in the other box with 1 2 3 change those number to the angles from inverse kinematics
% %%and we should have a linearised plant model
% for x = 1:4
%     G_lin(x,1) = subs(G(x,1),[a b c d g],[1 2 3 4 9.81]);
%     for y = 1:4
% D_lin(x,y) = subs(D(x,y),[a b c d g],[1 2 3 4 9.81]);
% C_lin(x,y) = subs(C(x,y),[a b c d g],[1 2 3 4 9.81]);
%     end
% end
% 
% vpa(D_lin);
% simplify(vpa(C_lin));
% simplify(vpa(G_lin));
% 
% tau = [t1;t2;t3;t4];
% input = inv(D_lin)*tau;
% state_terms = inv(D_lin)*(C_lin*q_vel+G_lin);
% 
% f1 = state_terms(1,1);
% f2 = state_terms(2,1);
% f3 = state_terms(3,1);
% f4 = state_terms(4,1);
% 
% Jacob_1 = [diff(f1,a_v), subs(diff(f1,b_v)), subs(diff(f1,c_v)), subs(diff(f1,d_v))];
% Jacob_2 = [diff(f2,a_v), subs(diff(f2,b_v)), subs(diff(f2,c_v)), subs(diff(f2,d_v))];
% Jacob_3 = [diff(f3,a_v), subs(diff(f3,b_v)), subs(diff(f3,c_v)), subs(diff(f3,d_v))];
% Jacob_4 = [diff(f4,a_v), subs(diff(f4,b_v)), subs(diff(f4,c_v)), subs(diff(f4,d_v))];
% 
% Jacob = [Jacob_1;Jacob_2;Jacob_3;Jacob_4];
% Jacob = vpa(Jacob);
% simplify(Jacob);


%Torque calculation
% tau = [t1;t2;t3;t4];
% %tau = D*q_acc + C*q_vel + G;
% % %% Jacobian linearization
% A = -inv(D)*(C*q_vel + G);
% 
% f1 = A(1,1);
% f2 = A(2,1);
% f3 = A(3,1);
% f4 = A(4,1);
% 
% Jacob_1 = [diff(f1,a), diff(f1,b), diff(f1,c), diff(f1,d)];
% Jacob_2 = [diff(f2,a), diff(f2,b), diff(f2,c), diff(f2,d)];
% Jacob_3 = [diff(f3,a), diff(f3,b), diff(f3,c), diff(f3,d)];
% Jacob_4 = [diff(f4,a), diff(f4,b), diff(f4,c), diff(f4,d)];
% 
% Jacob = [Jacob_1;Jacob_2;Jacob_3;Jacob_4]
% 
% 
% 
% 
% 
% %vpa(simplify(tau))
% %lin_J = [diff(f1,b)];%, diff(f1,b), diff(f1,c), diff(f1,c)]
% %Lin_J2 = [subs(lin_J(1,1),b,0)]
% %% Statespace
% %q_ddot = inv(D)*(tau - C*q_vel - G);
% % % taylor series
% % cos_b = taylor(cos(b),b,pi); %learise cos(b)
% % sin_b = taylor(sin(b),b,pi);
% % cos_bc = taylor(cos(b+c),[b c],[pi pi]);
% % sin_bc = taylor(sin(b+c),[b c],[pi pi]);
% % sin_bcd = taylor(sin(b+c+d),[b c d],[pi pi pi]);
% % cos_bcd = taylor(cos(b+c+d),[b c d],[pi pi pi]);
% % 
% % new_cos_b = subs(cos_b,b,3.2); %input the value of b you want ideally close to the point of linearisation 
% % new_sin_b = subs(sin_b,b,3.2);
% % new_cos_bc = subs(cos_bc,[b c],[3.2 3.2]);
% % new_sin_bc = subs(sin_bc,[b c],[3.2 3.2]);
% % new_sin_bcd = subs(sin_bcd,[b c d],[3.2 3.2 3.2]);
% % new_cos_bcd = subs(cos_bcd,[b c d],[3.2 3.2 3.2]);
% 
% % subs toolbox https://uk.mathworks.com/help/symbolic/subs.html
% % i think you can do it for matrices which will make life easier
% 
% % d11 = D(1,1)
% % d11 = subs(d11,cos(b),new_cos_b); %sub in the numeric calue of b into the equation
% % d11 = subs(d11,sin(b),new_sin_b);
% % d11 = subs(d11,cos(b + c),new_cos_bc);
% % d11 = subs(d11,sin(b + c),new_sin_bc);
% % d11 = subs(d11,sin(b+c+d),new_sin_bcd);
% % d11 = subs(d11,cos(b+c+d),new_cos_bcd);
% % d11 = simplify(d11)
% % 
% % 
% % d12 = D(1,2);
% % d12 = subs(d12,cos(b),new_cos_b); %sub in the numeric calue of b into the equation
% % d12 = subs(d12,sin(b),new_sin_b);
% % d12 = subs(d12,cos(b + c),new_cos_bc);
% % d12 = subs(d12,sin(b + c),new_sin_bc);
% % d12 = subs(d12,sin(b+c+d),new_sin_bcd);
% % d12 = subs(d12,cos(b+c+d),new_cos_bcd);
% % d12 = simplify(d12)
% % 
% % 
% % 
% % d13 = D(1,3);
% % d13 = subs(d13,cos(b),new_cos_b); %sub in the numeric calue of b into the equation
% % d13 = subs(d13,sin(b),new_sin_b);
% % d13 = subs(d13,cos(b + c),new_cos_bc);
% % d13 = subs(d13,sin(b + c),new_sin_bc);
% % d13 = subs(d13,sin(b+c+d),new_sin_bcd);
% % d13 = subs(d13,cos(b+c+d),new_cos_bcd);
% % d13 = simplify(d13)
% % 
% % 
% % 
% % d14 = D(1,4);
% % d14 = subs(d14,cos(b),new_cos_b); %sub in the numeric calue of b into the equation
% % d14 = subs(d14,sin(b),new_sin_b);
% % d14 = subs(d14,cos(b + c),new_cos_bc);
% % d14 = subs(d14,sin(b + c),new_sin_bc);
% % d14 = subs(d14,sin(b+c+d),new_sin_bcd);
% % d14 = subs(d14,cos(b+c+d),new_cos_bcd);
% % d14 = simplify(d14)
% 
% 
% 
% % d21 = D(2,1);
% % d22 = D(2,2);
% % d23 = D(2,3);
% % d24 = D(2,4);
% % d31 = D(3,1);
% % d32 = D(3,2);
% % d33 = D(3,3);
% % d34 = D(3,4);
% % d41 = D(4,1);
% % d42 = D(4,2);
% % d43 = D(4,3);
% % d44 = D(4,4);
   
            



