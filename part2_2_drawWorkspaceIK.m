%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%
close all
clc

rotation_alpha = 30;
DRAW_STEP = 3;
PLATFORM_R = 130;
BASE_R = 290;
LENGTH_RA = 170;
LENGTH_L = 130;

deg2rad = pi / 180;

rotation_alpha = rotation_alpha * deg2rad;
rotationBtoC = [cos(rotation_alpha) -sin(rotation_alpha) 0; sin(rotation_alpha) cos(rotation_alpha) 0; 0 0 1];

vectorBtoPB = zeros(3,3);
vectorCtoPP = zeros(3,3);
vectorBtoPP = zeros(3,3);
vectorPBtoPP = zeros(3,3);
variable_e = zeros(3,3);
solution_theta = zeros(3,2);
achievable_position = true;
zero_dot = true;
for position_X = -(LENGTH_RA + LENGTH_L) : DRAW_STEP : (LENGTH_RA + LENGTH_L)
    for position_Y = -(LENGTH_RA + LENGTH_L) : DRAW_STEP : (LENGTH_RA + LENGTH_L)
        vectorBtoC = [position_X; position_Y; 0];
        for i = 1 : 3
            vectorBtoPB(1:3, i) = [BASE_R * cos(-pi / 6 + 2 * i * pi / 3 ); BASE_R * sin(-pi / 6 + 2 * i * pi / 3); 0];
            vectorCtoPP(1:3, i) = [PLATFORM_R * cos(-pi / 6 + 2 * i * pi / 3); PLATFORM_R * sin(-pi / 6 + 2 * i * pi / 3); 0];
            vectorBtoPP(1:3, i) = rotationBtoC * vectorCtoPP(1:3, i) + vectorBtoC;
            vectorPBtoPP(1:3, i) = vectorBtoPP(1:3, i) - vectorBtoPB(1:3, i);
            variable_e(1:3, i) = [-2 * vectorPBtoPP(2, i) * LENGTH_RA; -2 * vectorPBtoPP(1, i) * LENGTH_RA; ...
                                    vectorPBtoPP(2, i)^2 + vectorPBtoPP(1, i)^2 + LENGTH_RA^2 - LENGTH_L^2];
            variable_D = variable_e(1,i)^2 + variable_e(2,i)^2 - variable_e(3,i)^2;
            variable_D = roundn(variable_D, -10);
            if variable_D < 0
                achievable_position = false;
                break;
            else
                zero_dot = false;
            end            
        end
        if achievable_position == true && zero_dot == false
            plot(vectorBtoC(1),vectorBtoC(2),'.')
            hold on
        else
            achievable_position = true;
        end
    end
end

if zero_dot == true
    disp(['The end-effector can not achieve any position with angle ',num2str(rotation_alpha / deg2rad),' !']) 
else
    title(['workspace for an orientation alpha:',num2str(rotation_alpha/deg2rad)])
    xlabel('x-axis')
    ylabel('y-axis')
    axis equal
end
