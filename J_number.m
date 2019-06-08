%%%%%%%%%%%%%%%%%% LU QIAN %%%%%%%%%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%% November 2018 %%%%%%%%%%%%%%

%% e.g. input (200,-150,50,270)

%%
function y = J_number(px,py,pz,theta_pitch)

%% goal
Xg = [px py pz theta_pitch*pi/180]';
%% Links Length
l0 = 63.5 ;
l1 = 0 ;
l2 = 152.4 ;
l3 = 157.48 ; 
l4 = 34.29 ;
l5 = 66.04 ;
%% angle boundary
theta_bound(1,:) = [0 pi];
theta_bound(2,:) = [0 pi];
theta_bound(3,:) = [0 pi*5/6];
theta_bound(4,:) = [-pi/2 pi/2];
%%theta_bound(5,:) = [-pi/2 pi/2];

% syms q1_lower q1_upper q2_lower q2_upper q3_lower q3_upper q4_lower q4_upper real;
% theta_bound(1,:) = [q1_lower q1_upper];
% theta_bound(2,:) = [q2_lower q2_upper];
% theta_bound(3,:) = [q3_lower q3_upper];
% theta_bound(4,:) = [q4_lower q4_upper];
%% get Jacobian equation
% Trigonometric abbreviations
syms q1 q2 q3 q4 real;
c1 = cos(q1);
s1 = sin(q1);

c2 = cos(q2);
s2 = sin(q2);

c23 = cos(q2+q3);
s23 = sin(q2+q3);

c234 = cos(q2+q3+q4);
s234 = sin(q2+q3+q4);

% current EE position



xt = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23+(l4+l5)*c234).*c1;
yt = ((l0+l1)*cos(pi/2)+l2*c2+l3*c23+(l4+l5)*c234).*s1;
zt = (l0+l1)*sin(pi/2)+l2*s2+l3*s23+(l4+l5)*s234; 
theta_pitch = q2+q3+q4;

X = [xt yt zt theta_pitch]';
theta = [q1 q2 q3 q4]';

dX = Xg-X;


for i = 1:4
    for j =1:4
        Jacob(i,j) = diff(X(i),theta(j)) ;
    end
end
Jacob_inv = inv(Jacob);
%% numerical iteration
% give start number £¨in this case, I chose q1 q2 q3 q4 all equal 45 degrees as start position, to avoid no solution of Jacobian£©

theta = [pi/4 pi/4 pi/4 pi/4]'; 
T_start = FK_get_T0e(theta(1),theta(2),theta(3),theta(4),0); 
X = [T_start(1,4) T_start(2,4) T_start(3,4) 3/4*pi];
theta_temp = theta/pi*180;


%% tolerance
tol = 1;
dist = norm(X(1:3)- Xg(1:3));
count = 0;
%%
if norm(Xg(1:3))<= l0+l1+l2+l3+l4+l5
while dist>tol
    q1 = theta(1);
    q2 = theta(2);
    q3 = theta(3);
    q4 = theta(4);
   Jacob_inv_num = eval(Jacob_inv);
   dX_num = eval(dX);
cond = theta + Jacob_inv_num*dX_num;
for i2 =1:4
    if cond(i2) < theta_bound(i2,1)
        theta(i2) = theta_bound(i2,1);
    elseif cond(i2) > theta_bound(i2,2)
        theta(i2) = theta_bound(i2,2);
    else
        theta(i2) = cond(i2);
    end
end
T = FK_get_T0e(theta(1)*180/pi,theta(2)*180/pi,theta(3)*180/pi,theta(4)*180/pi,0);
X = [T(1,4) T(2,4) T(3,4) (theta(2)+theta(3)+theta(4))*180/pi]';
dist = norm(X(1:3)- Xg(1:3));
theta_temp = theta/pi*180;

count = count+1;
if count>50
    disp('cannot calculate')
    return
end
end
y = theta/pi*180;
fprintf('q1 = %.1f ,',y(1))
fprintf('q2 = %.1f ,',y(2))
fprintf('q3 = %.1f ,',y(3))
fprintf('q4 = %.1f ,',y(4))
fprintf('q5 = %.1f \n',0)
else
    disp('cannot reach')
end



