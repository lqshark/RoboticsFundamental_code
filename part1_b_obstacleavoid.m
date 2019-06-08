%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%

clear all
close all
clc

%% input AB obstacle (consider about avoid in X,Y directions) 

x_obstacle = 150;
y_obstacle = -150;
r_obstacle = 5;

h_obstacle = 100; % height

figure (1)
set(1,'position',[500 200 560 420])

[x,y,z]=cylinder(r_obstacle);
surf(x+x_obstacle,y+y_obstacle,h_obstacle*z)
hold on
grid on
axis([-100 450 -300 200 0 500]) % axis range

%% Links Length 
l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.5 ; 
l4 = 34.3 ;
l5 = 66.0 ;

theta_pitch = 270;
t = 0.01;
%% straight line path

length = 2;
u = 2; % constant speed m/s

path(1,:) = [200 -150 50]; %A
path(2,:) = [100 -150 50]; %B
path(3,:) = [100 -50 50]; %C
path(4,:) = [200 -50 50]; %D
path(5,:) = [150 -100 50]; %E
path(6,:) = [200 -150 50]; %A

p_avoid = [x_obstacle-4*r_obstacle y_obstacle 50;
    x_obstacle-2*r_obstacle y_obstacle 50;
    x_obstacle -150-4*r_obstacle 50;
    x_obstacle+2*r_obstacle y_obstacle 50;
    x_obstacle+4*r_obstacle y_obstacle 50];
x = p_avoid(:,1);
y = p_avoid(:,2);
p = polyfit(x,y,3);
z = polyval(p,x);


%% put each EE position into IK function to get joint q1-5 (choose 1 solution for each)
q = zeros(8,5);

for j_q = 1:5
temp = IK(path(j_q,1),path(j_q,2),path(j_q,3),theta_pitch);
q(j_q+1,:) = temp(1,:);
end
q(7,:) = q(2,:);
%%
T_store = zeros(8,3);
Tpt_store = zeros(7,20,3);
for j0 = 1:8
T_temp = FK_get_T0e(q(j0,1),q(j0,2),q(j0,3),q(j0,4),q(j0,5));
T_store(j0,:) = [T_temp(1,4) T_temp(2,4) T_temp(3,4)];
end

    n2 = 20;
%     plot3(T_store(1,1),T_store(1,2),T_store(1,3),'*');
%     text(T_store(1,1)+ n2,T_store(1,2)+ n2,T_store(1,3)+ n2,'Start&End','fontsize',7) ;
    plot3(T_store(2,1),T_store(2,2),T_store(2,3),'*');
    text(T_store(2,1)+ n2,T_store(2,2)- n2,T_store(2,3),'A','fontsize',7) ;

    plot3(T_store(3,1),T_store(3,2),T_store(3,3),'*');
    text(T_store(3,1)- n2,T_store(3,2)- n2,T_store(3,3),'B','fontsize',7) ;
    plot3(T_store(4,1),T_store(4,2),T_store(4,3),'*');
    text(T_store(4,1)- n2,T_store(4,2)+ n2,T_store(4,3),'C','fontsize',7) ;
    plot3(T_store(5,1),T_store(5,2),T_store(5,3),'*');
    text(T_store(5,1)+ n2,T_store(5,2)+ n2,T_store(5,3),'D','fontsize',7) ;
    plot3(T_store(6,1),T_store(6,2),T_store(6,3),'*');
    text(T_store(6,1)+ n2,T_store(6,2),T_store(6,3),'E','fontsize',7) ;

for j1 = 2:size(path,1)
dist(j1-1) = norm(path(j1,:)-path(j1-1,:));
for j2 = 0:dist(j1-1)/length
    via = path(j1-1,:)+j2*(path(j1,:)-path(j1-1,:))/(dist(j1-1)/length);
    
    
    
    if j1 == 2 && via(1)<= x_obstacle+4*r_obstacle && via(1) >= x_obstacle-4*r_obstacle
        error_value = polyval(p,p_avoid(1,1))-via(2);
        via(2) = polyval(p,via(1))-error_value;
    end

    theta_temp = IK(via(1),via(2),via(3),270)*pi/180;
    theta = theta_temp(1,:);
    
    
    c1 = cos(theta(1));
    s1 = sin(theta(1));
    c2 = cos(theta(2));
    s2 = sin(theta(2));
    c23 = cos(theta(2)+theta(3));
    s23 = sin(theta(2)+theta(3));
    c234 = cos(theta(2)+theta(3)+theta(4));
    s234 = sin(theta(2)+theta(3)+theta(4));
      
    
    
    x0 = zeros(1) ;
    y0 = zeros(1) ;
    z0 = zeros(1) ;
    
    x1 = l0*cos(pi/2).*c1 ;
    y1 = l0*cos(pi/2).*s1 ;
    z1 = l0*sin(pi/2);
    
    x2 = (l0+l1)*cos(pi/2).*c1 ;
    y2 = (l0+11)*cos(pi/2).*s1;
    z2 = (l0+l1)*sin(pi/2) ;
    
    x3 = ((l0+l1)*cos(pi/2)+l2*c2).*c1 ;
    y3 = ((l0+l1)*cos(pi/2)+l2*c2).*s1 ;
    z3 = (l0+l1)*sin(pi/2)+l2*s2 ;
    
    x4 = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23).*c1 ;
    y4 = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23).*s1 ;
    z4 = (l0+l1)*sin(pi/2)+l2*s2+l3*s23 ;
    
    x5 = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23+l4*c234).*c1 ;
    y5 = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23+l4*c234).*s1 ;
    z5 = (l0+l1)*sin(pi/2)+l2*s2+l3*s23+l4*s234 ;

    xx = [x0; x1; x2; x3; x4; x5; via(1) ] ;
    yy = [y0; y1; y2; y3; y4; y5; via(2) ] ;
    zz = [z0; z1; z2; z3; z4; z5; via(3) ] ;


    plot3(via(1),via(2),via(3),'r.','MarkerSize',0.5)

    arm = plot3(xx,yy,zz,'k.-','Linewidth',1,'MarkerSize',15,'tag','arm');

    axis equal
    axis([-100 450 -300 200 0 500]) % axis range
    
    xlabel('x (mm)') ; ylabel('y (mm)') ; zlabel('z (mm)') ;
   

    pause(t) % considering accelaration, this t can be a vector.
    h = findobj('type','line','tag','arm');
    delete(h);
end
end
plot3(xx,yy,zz,'k.-','Linewidth',1,'MarkerSize',15)