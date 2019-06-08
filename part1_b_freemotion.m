%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%

clear all
close all
clc
%% Links Length 
l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.5 ; 
l4 = 34.3 ;
l5 = 66.0 ;

%% desired path of EE 
theta_pitch = 270;
path = zeros(6,3);

path(1,:) = [200 -150 50]; %A
path(2,:) = [100 -150 50]; %B
path(3,:) = [100 -50 50]; %C
path(4,:) = [200 -50 50]; %D
path(5,:) = [150 -100 50]; %E
path(6,:) = [200 -150 50]; %A

figure (1)
set(1,'position',[0 200 560 420]) % position and size
plot3(path(:,1),path(:,2),path(:,3))
% axis([0 250 -200 0 49.999995 50.000005]) % axis range
hold on

%% put each EE position into IK function to get joint q1-5 (choose 1 solution for each)
q = zeros(8,5);

for j = 1:5
temp = IK(path(j,1),path(j,2),path(j,3),theta_pitch);
q(j+1,:) = temp(1,:);
end
q(7,:) = q(2,:);
%%
T_store = zeros(8,3);
Tpt_store = zeros(7,20,3);
%% FK
for i1 = 1:8
T_temp = FK_get_T0e(q(i1,1),q(i1,2),q(i1,3),q(i1,4),q(i1,5));
T_store(i1,:) = [T_temp(1,4) T_temp(2,4) T_temp(3,4)];
end


plot3(T_store(2:7,1),T_store(2:7,2),T_store(2:7,3),'r')
legend('desired path','IK results path')
xlabel('x (mm)') ; ylabel('y (mm)') ; zlabel('z (mm)') ;
%% free motion path
n = 20;
t = 0.01;
for i2 = 1:7
q1(i2,:) = linspace(q(i2,1),q(i2+1,1),n)'*pi/180 ;
q2(i2,:) = linspace(q(i2,2),q(i2+1,2),n)'*pi/180 ;
q3(i2,:) = linspace(q(i2,3),q(i2+1,3),n)'*pi/180 ;
q4(i2,:) = linspace(q(i2,4),q(i2+1,4),n)'*pi/180 ;
q5(i2,:) = zeros(n,1) *pi/180 ;

for i3 = 1:n
    grid on
    hold on
    
    c1 = cos(q1(i2,i3));
    s1 = sin(q1(i2,i3));
    c2 = cos(q2(i2,i3));
    s2 = sin(q2(i2,i3));
    c23 = cos(q2(i2,i3)+q3(i2,i3));
    s23 = sin(q2(i2,i3)+q3(i2,i3));
    c234 = cos(q2(i2,i3)+q3(i2,i3)+q4(i2,i3));
    s234 = sin(q2(i2,i3)+q3(i2,i3)+q4(i2,i3));
    
    figure (2)
    set(2,'position',[500 200 560 420])
    
     
    n2 = 20;
    plot3(T_store(1,1),T_store(1,2),T_store(1,3),'*');
    text(T_store(1,1)+ n2,T_store(1,2)+ n2,T_store(1,3)+ n2,'Start&End','fontsize',7) ;
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

    
    Tpt = FK_get_T0e(q1(i2,i3)/pi*180 ,q2(i2,i3)/pi*180 ,q3(i2,i3)/pi*180 ...
                     ,q4(i2,i3)/pi*180 ,q5(i2,i3)/pi*180);
    Tpt_store(i2,i3,:) = [Tpt(1,4) Tpt(2,4) Tpt(3,4)];
    xx = [x0; x1; x2; x3; x4; x5; Tpt_store(i2,i3,1) ] ;
    yy = [y0; y1; y2; y3; y4; y5; Tpt_store(i2,i3,2) ] ;
    zz = [z0; z1; z2; z3; z4; z5; Tpt_store(i2,i3,3) ] ;

    plot3(Tpt_store(i2,i3,1),Tpt_store(i2,i3,2),Tpt_store(i2,i3,3),'r.','MarkerSize',0.5)
   
    arm = plot3(xx,yy,zz,'k.-','Linewidth',1,'MarkerSize',15,'tag','arm');
    
    axis equal

        
    xlabel('x (mm)') ; ylabel('y (mm)') ; zlabel('z (mm)') ;
   
    axis([-100 450 -300 200 0 500]) % axis range
    pause(t) % considering accelaration, this t can be a vector.
    h = findobj('type','line','tag','arm');
    delete(h);
    
end
end
plot3(xx,yy,zz,'k.-','Linewidth',1,'MarkerSize',15)

