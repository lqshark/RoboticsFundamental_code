%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%


clear all
close all
clc
%% about home position
% in order to simplify calculation and garantee the constant sign of joint
% variables (q1,q2,q3)
%% Links Lengths (mm)

l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.5 ; 
l4 = 34.3 ;
l5 = 66.0 ;


%% Workspace 3D(2D can also obtain in this figure)

% q1 = 0,180
% q2 = 0,180
% q3 = 0,150
% q4 = (-90,90) +90
% q5 = -90,90
% but q5 cannot affect the position

% proximal convention

q1 = 0:pi/180:180*pi/180;
q2 = 0:pi/180:180*pi/180;
q3 = 0:pi/180:150*pi/180;
q4 = -90*pi/180+90*pi/180:pi/180:90*pi/180+90*pi/180;
q5 = -90*pi/180:pi/180:90*pi/180;

n1 = 20; % between length of q1245
n2 = 15; % between length of q3
e = 0; % for counting
work = zeros(3,10*10*11*10*10) ; % reserving variables

for i1 = 1:length(q1)/n1+1
    for i2 = 1:length(q2)/n1+1
        for i3 = 1:length(q3)/n2+1
            for i4 = 1:length(q4)/n1+1
                for i5 = 1:length(q5)/n1+1
                    
                    qw1 = q1(1+n1*(i1-1));
                    qw2 = q2(1+n1*(i2-1));
                    qw3 = q3(1+n2*(i3-1));
                    qw4 = q4(1+n1*(i4-1));
                    qw5 = q5(1+n1*(i5-1));
                    
                    DH = [0 0 l0 qw1;
                        0 pi/2 0 qw2;
                        l2 0 0 qw3;
                        l3 0 0 qw4;
                        0 pi/2 l4 qw5;
                        0 0 l5 0];
                    
                    T = 1;
                    
                    for i = 1:6
                        a = DH(i,1);
                        alpha = DH(i,2);
                        d = DH(i,3);
                        theta = DH(i,4);
                        
                        Ti = [cos(theta) -sin(theta) 0 a;
                            sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
                            sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
                            0 0 0 1];
                        
                        T = T * Ti;
                    end

                    e = e + 1;
                    work(1,e)=T(1,4);
                    work(2,e)=T(2,4);
                    work(3,e)=T(3,4);
%                     plot3(T(1,4),T(2,4),T(3,4),'.')  % plot in the loop can be colourful 
                end
            end
        end
    end
end
plot3(work(1,:),work(2,:),work(3,:),'.')
hold on
grid on
title('workspace') ; 
xlabel('x (mm)') ; 
ylabel('y (mm)') ;
zlabel('z (mm)') ;
axis equal