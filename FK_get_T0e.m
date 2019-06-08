%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%

function y = FK_get_T0e(q1,q2,q3,q4,q5)

%% input angle range
% q1 = 0,180
% q2 = 0,180
% q3 = 0,150
% q4 = -90,90
% q5 = -90,90

%% known parameters(length:mm)
l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.5 ; 
l4 = 34.3 ;
l5 = 66.0 ;

q1 = q1 *pi/180 ;
q2 = q2 *pi/180 ;
q3 = q3 *pi/180 ;
q4 = q4 *pi/180 +90*pi/180;
q5 = q5 *pi/180 ;

%%
DH = [0 0 l0 q1;
    0 pi/2 0 q2;
    l2 0 0 q3;
    l3 0 0 q4;
    0 pi/2 l4 q5;
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

T0e = T;
y = T0e;