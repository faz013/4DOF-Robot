clear all

l1 = 19;
l2 = 18;
l3 = 18;
l4 = 9;

phi = 0;
res = 100;
%define lengths
t1=linspace(0,180,res)*pi/180; %joint angles
t2=linspace(0,130,res)*pi/180;
t3=linspace(-130,130,res)*pi/180;
%t4=linspace(-40,220,res)*pi/180;

[T1,T2,T3]=ndgrid(t1,t2,t3); %only need these 3 since t2 + t3 + t4 will always equal 180  

xM = cos(T1).*(l2.*cos(T2)+l3.*cos(T2+T3)+l4.*cos(phi)); % FK equations
yM = sin(T1).*(l2.*cos(T2)+l3.*cos(T2+T3)+l4.*cos(phi));
zM = (l1 + l2.*sin(T2) + l3.*sin(T2+T3)+ l4.*sin(phi));

% figure; %3D plot
% plot3(xM(:),yM(:), zM(:),'.') %zM(:)
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')


it = size(xM(:));

x = xM(:);
y = yM(:);
z = zM(:);

t = 1;
k = 1;
x_val = zeros(it(1,1),1);
y_val = zeros(it(1,1),1);
z_req = 0;

while t<it(1,1) %generating a plot of only x-y axis there is probably a quicker way

   if z(t,1) < 10
       if z(t,1)> 9
        x_val(k,1) = x(t,1);
        y_val(k,1) = y(t,1);
        k = k+1;
       end
   end
    t = t + 1;
    Percentage = (t*100)/it(1,1)
end

axis equal
plot(x_val,y_val)
axis equal
xlabel('x (cm)')
ylabel('y (cm)')
% 
% while t<it(1,1) %generating a plot of only x-y axis there is probably a quicker way
% 
%    if abs(z(t,1) - z_req) < 0.1
%         x_val(k,1) = x(t,1);
%         y_val(k,1) = y(t,1);
%         k = k+1;
%    end
%     t = t + 1;
%     Percentage = (t*100)/it(1,1)
% end
% 
% 
% figure; %the plot
% plot(x_val,y_val,'x')
% xlabel('x')
% ylabel('y')
