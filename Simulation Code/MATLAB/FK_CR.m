%% Generating the joint angles from the positions given

%clear all
%enter the joint lengths
l1 = 19;
l2 = 18;
l3 = 18;
l4 = 8;


thetha = 0;
cx = 30; %the center points of the circle
cy = 0;
radius = 10;
px = cx-radius*cos(thetha);
py = cy-radius*sin(thetha);
pz = 15;
phi = 0; %end-effector orientation


n = 100; % points to be made
i = 1;
q1 = zeros(n-1,1);
q2 = zeros(n-1,1);
q3 = zeros(n-1,1);
q4 = zeros(n-1,1);

while i < n
q1(i,1) = atan2(py,px); %inverse kinematic equations

eq1 = px/cos(q1(i,1)) - l4*cos(phi);
eq2 = pz-l1-l4*sin(phi);

c3 = (eq1^2+eq2^2 - l2*l2 - l3*l3)/(2*l2*l3);
s3 = -sqrt(abs(1-c3*c3));

q3(i,1) = atan2(s3,c3);

k1 = l2+l3*cos(q3(i,1));
k2 = l2*sin(q3(i,1));

q2(i,1) = atan2(eq2,eq1)-atan2(k2,k1);

q4(i,1) = phi - q2(i,1)-q3(i,1);
i = i+1;
px = cx+radius*cos(thetha);
py = cy+radius*sin(thetha);
thetha = thetha + pi/40;
end

%ik = [q1;q2;q3;q4]

%% Trigonometric abbreviations
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);

c12 = cos(q1+q2);
c23 = cos(q2+q3);
c234 = cos(q2+q3+q4);


s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);

s12 = sin(q1+q2);
s23 = sin(q2+q3);
s234 = sin(q2+q3+q4);

% Tip position
%These equations are derived from the Forward Kinematic model of the robot
xt = c1.*(l2*c2+l3*c23+l4*c234) ;
yt = s1.*(l2*c2+l3*c23+l4*c234) ;
zt = l1 + l2*s2 + l3*s23 + l4*s234;
pt = [ xt yt zt] ;
% 
%%
%Plot the trajectory of the end-effector
 figure (1)
% % 
% % 
 %plot3(-pt(1,1)+10,-pt(1,2)+10,pt(1,3),'bo')       % plot the first position of the robot's end effector
 hold on
 plot3(-pt(2:n-1,1),-pt(2:n-1,2),pt(2:n-1,3),'bo')       % plot the 3 following positions of the robot's end effector
 title('End-Effector Position') ; xlabel('x (cm)') ; ylabel('y (cm)');zlabel('z (cm)') ;
 grid on
% 
% 
%Plot the robotic arm, in different positions
figure (2) 
%set(2,'position',[116 190 560 420])

x1 = zeros(n-1,1) ;
y1 = zeros(n-1,1) ;
z1 = zeros(n-1,1); 

x2 = zeros(n-1,1) ;
y2 = zeros(n-1,1) ;
z2 = l1*ones(n-1,1);



x3 = l2*c1.*c2;
y3 = l2*c2.*s1;
z3 = l1 + l2.*s2;


x4 = c1.*(l3*c23+l2*c2);
y4 = s1.*(l3*c23+l2*c2);
z4 = l1 + l3*s23 +l2*s2 ;
hold off
for i = 1:n-1
    xx = [ -x1(i); -x2(i);-x3(i);-x4(i); -pt(i,1) ] ;
    yy = [ -y1(i); -y2(i);-y3(i);-y4(i); -pt(i,2) ] ;
    zz = [ z1(i); z2(i);z3(i);z4(i); pt(i,3) ];

    plot3(xx,yy,zz,'ko-','Linewidth',2)
    axis equal
    hold on
    grid on
      caz = 35.7661;
        cel = 26.6341;
        v = [100 50 100];;
[caz,cel] = view(v);
    xlabel('x (mm)') ; ylabel('y (mm)'); zlabel('z (mm') ;
   % text(pt(1,1),pt(1,2),'x') ; text(pt(1,1) + 0.002,pt(1,2) + 0.002,'ptStart') ;
    %text(pt(4,1),pt(4,2),'x') ; text(pt(4,1) + 0.002,pt(4,2) + 0.002,'ptEnd') ;
    %axis([-10 100 -10 100 -10 100])
    %title("Kinematic Model showing joint positions")
    pause(0.01)
    hold off
    pause(0.01)
          % plot the first position of the robot's end effecton
    
end
% hold on
% plot3(-pt(1,1)+10,-pt(1,2)+10,pt(1,3),'bo')       % plot the first position of the robot's end effector
%  plot3(-pt(2:n-1,1),-pt(2:n-1,2),pt(2:n-1,3),'bo')       % plot the 3 following positions of the robot's end effector
%  title('End-Effector Position') ; xlabel('x (cm)') ; ylabel('y (cm)');zlabel('z (cm)') ;
%  grid on
