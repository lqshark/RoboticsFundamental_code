%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%

clear all
close all
clc
%% design parameters (mm)
rA = 170;
L = 130;
rplat = 130;
rbase = 290;
%% angular joint positions
% PB1 = pi/2;
% PB2 = pi+pi/6;
% PB3 = 2*pi-pi/6;
% PP1 = pi/2;
% PP2 = pi+pi/6;
% PP3 = 2*pi-pi/6;
%% input

% a = 30*pi/180;
% xc = 10;
% yc = 5;

prompt01 = 'X(mm) =  ';
xc = input(prompt01);
prompt02 = 'Y(mm) =  ';
yc = input(prompt02);
prompt03 = 'alpha(deg) =  ';
alpha = input(prompt03);
a = alpha*pi/180;


Ra = [cos(a) -sin(a) 0
      sin(a)  cos(a) 0
           0       0 1];
C = [xc yc 0]';

%% Declare variable
BPP = zeros(3,3);
BPB = zeros(3,3);
Mia = zeros(3,3);
Mib = zeros(3,3);
%% get PBiPPi
BC = C;
for i = 1:3
CPPi = [rplat*cos(-5*pi/6+(i-1)*2*pi/3) rplat*sin(-5*pi/6+(i-1)*2*pi/3) 0]';
BPP(:,i) = Ra*CPPi+BC;
BPB(:,i) = [rbase*cos(-5*pi/6+(i-1)*2*pi/3) rbase*sin(-5*pi/6+(i-1)*2*pi/3) 0]';

PBiPPi = BPP(:,i)-BPB(:,i);
xppi = PBiPPi(1);
yppi = PBiPPi(2);

% xppi yppi wrt {PB}
e1 = -2*yppi*rA;
e2 = -2*xppi*rA;
e3 = xppi^2+ yppi^2+rA^2-L^2;

if e1^2+e2^2-e3^2 < 0 
    disp('no solutions!')
    return
end
    
t1 = (-e1+sqrt(e1^2+e2^2-e3^2))/(e3-e2);
t2 = (-e1-sqrt(e1^2+e2^2-e3^2))/(e3-e2);
theta_a = 2*atan(t1);
theta_b = 2*atan(t2);

% get M
Mia(:,i) = [rA*cos(theta_a)+rbase*cos(-5*pi/6+(i-1)*2*pi/3);
    rA*sin(theta_a)+rbase*sin(-5*pi/6+(i-1)*2*pi/3);
    0];
Mib(:,i) = [rA*cos(theta_b)+rbase*cos(-5*pi/6+(i-1)*2*pi/3);
    rA*sin(theta_b)+rbase*sin(-5*pi/6+(i-1)*2*pi/3);
    0];
end

for i=1:3
    plot(BPB(1,i),BPB(2,i),'ro')
    hold on
    plot(BPP(1,i),BPP(2,i),'ro')
    plot(Mia(1,i),Mia(2,i),'ro')
    plot(Mib(1,i),Mib(2,i),'r*')
    plot([BPP(1,i) Mia(1,i)],[BPP(2,i) Mia(2,i)],'b')
    plot([BPB(1,i) Mia(1,i)],[BPB(2,i) Mia(2,i)],'g')
    plot([BPP(1,i) Mib(1,i)],[BPP(2,i) Mib(2,i)],'b--')
    plot([BPB(1,i) Mib(1,i)],[BPB(2,i) Mib(2,i)],'g--')
end

plot([BPB(1,:) BPB(1,1)],[BPB(2,:) BPB(2,1)],'b')
plot([BPP(1,:) BPP(1,1)],[BPP(2,:) BPP(2,1)],'r')
grid on
xlabel('x-axis')
ylabel('y-axis')
title(['Data - Cartesian position X(mm),Y(mm),alpha(deg) = ' ...
    ,num2str(xc),' ',num2str(yc),' ',num2str(alpha)],'FontSize',9)
axis equal
