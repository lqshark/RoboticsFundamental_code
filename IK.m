%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%

function y = IK(px,py,pz,theta_pitch)

%% input desired Position and Orientation of end-effector
% Position : px,py,pz
% Orientation : theta_pitch,(theta_roll,theta_yaw)

%% Links Length 
% the inaccurate value of length will lead to the deviation of joint variable results
l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.5 ; 
l4 = 34.3 ;
l5 = 66.0 ;

%% Declare variables
syms q1 q2 q3 q4 q5 no q234;
theta_roll = 0; % equals to q5, only affects orientation, no effects  
                % on position, so can be any value in angle range [-90,90]
% theta_yaw = q1 can obtain by px and py

%% %%%%%%%%%%%% Inverse Kinematics of Lyxnmotion Robot %%%%%%%%%%%%%%%%%

q5 = theta_roll *pi/180;
q234 = theta_pitch *pi/180;
c234 = cos(q234);
s234 = sin(q234);
%% Find q1
q1 = atan2(py,px);
if q1 == 0 || q1 == pi
    q1 = [0 pi]';
else
    q1 = [q1 q1+pi]'; % may not in angle range, need to judge at final part
end

%% reserving variables
q2a = zeros(2,1);
q2b = zeros(2,1);
q3a = zeros(2,1);
q3b = zeros(2,1);
q4a = zeros(2,1);
q4b = zeros(2,1);

for i = 1:2 % for 2 values of q1
    
% find T01 with DH (row1)    
DH = [0 0 l0 q1(i)];

a = DH(1,1);
alpha = DH(1,2);
d = DH(1,3);
theta = DH(1,4);

T01 = [cos(theta) -sin(theta) 0 a;
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
    0 0 0 1];

% use T01 T0e for getting T1e
T1e = T01 \ [px py pz 1]';


x1e = T1e(1);
z1e = T1e(3);
    z14 = z1e-(l4+l5)*s234;
if x1e>=0
    x14 = x1e-(l4+l5)*c234;
else
    x14 = x1e+(l4+l5)*c234;
end

%% Find q3-2-4(need x14,z14,s234,c234)

c3 = (x14.^2+z14.^2-l2^2-l3^2)/(2*l2*l3);
if c3 > -1 && c3 < 1
    s3a = +sqrt(1-c3^2);
    s3b = -sqrt(1-c3^2);
    c3a = c3;
    c3b = c3;
elseif abs(c3)-1 < 0.001
    s3a = 0;
    s3b = 0;
    c3a = c3;
    c3b = c3;
else % no solution for this q1
    q3a(i) = 9000;
    q3b(i) = 9000;
    q2a(i) = 9000;
    q2b(i) = 9000;
    q4a(i) = 9000;
    q4b(i) = 9000;
    continue
end
q3a(i) = atan2(s3a,c3);
q3b(i) = atan2(s3b,c3);
    
q2a(i) = atan2(z14,x14) - atan2(l3*s3a,l2+l3*c3a);
q2b(i) = atan2(z14,x14) - atan2(l3*s3b,l2+l3*c3b);

q4a(i) = q234 - q2a(i) - q3a(i) ;
q4b(i) = q234 - q2b(i) - q3b(i) ;
end
q_results = [q1(1) q2a(1) q3a(1) q4a(1) q5;
             q1(1) q2b(1) q3b(1) q4b(1) q5;
             q1(2) q2a(2) q3a(2) q4a(2) q5;
             q1(2) q2b(2) q3b(2) q4b(2) q5];
Q_results = q_results*180/pi;
%% output
count = 0;
value_error = 0;
for j = 1:4
    if Q_results(j,2) == 9000
        continue
    elseif Q_results(j,1)>=0 - value_error && Q_results(j,1)<=180 + value_error && ...
            Q_results(j,2)>=0 - value_error && Q_results(j,2)<=180 + value_error && ...
            Q_results(j,3)>=0 - value_error && Q_results(j,3)<=150 + value_error && ...
            Q_results(j,4)>=-90 - value_error && Q_results(j,4)<=90 + value_error && ...
            Q_results(j,5)>=-90 - value_error && Q_results(j,5)<=90 + value_error 
        count = count+1;
        sol(count,:) = Q_results(j,:);  
        fprintf('solution %.0f : ',count)
        fprintf('q1 = %.1f ,',Q_results(j,1))
        fprintf('q2 = %.1f ,',Q_results(j,2))
        fprintf('q3 = %.1f ,',Q_results(j,3))
        fprintf('q4 = %.1f ,',Q_results(j,4))
        fprintf('q5 = %.1f \n',Q_results(j,5))
    else
        continue
    end
end
if count == 1
    disp('In conclusion, only 1 solution is obtained.')
    y = sol;
elseif count == 0
    disp('No solution.')
else
fprintf('In conclusion, %.0f solutions are obtained. \n',count)
y = sol;
end
disp('[In this case, we choose q5 = 0 degree.')
disp('q5 only affects orientation, have no effects on position, so it can be any value in angle range[-90,90].')
disp('Additionally, the deviation of joint variable results are caused by the inaccurate value of length.]')
