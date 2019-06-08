%%%%%%%%  LU QIAN %%% YUAN JIALIN %%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%
function inverseKinematics(goal_position_X, goal_position_Y, goal_position_Z, angle_pitch, angle_roll)

    global THETA1_MIN; global THETA1_MAX;
    global THETA2_MIN; global THETA2_MAX;
    global THETA3_MIN; global THETA3_MAX;
    global THETA4_MIN; global THETA4_MAX;
    global THETA5_MIN; global THETA5_MAX;
    
    deg2rad = pi / 180;
%     THETA4 = 0 : 360;
    THETA4 = angle_pitch + 180;
    ALLOWABLE_ERROR = 0.0000000001;
    THETA1_MIN = 0; THETA1_MAX = 180;
    THETA2_MIN = 0; THETA2_MAX = 180;
    THETA3_MIN = -180; THETA3_MAX = 0;
    THETA4_MIN = -90; THETA4_MAX = 90;
    THETA5_MIN = 0; THETA5_MAX = 180;
    LINK_LENGTH = [63.5, 152.4, 157.48, 34.29, 66.04];
%     LINK_LENGTH = [10, 10, 10, 10, 10];

    theta1 = [atan2(goal_position_Y, goal_position_X); rem(atan2(goal_position_Y, goal_position_X) + pi, 2 * pi)];
    theta2 = zeros(1, 2);
    theta3 = zeros(1, 2);
    THETA4 = THETA4 * deg2rad;
    resultAngles = zeros(length(THETA4) * 4, 4);
    
    array_index = 1;
    for i = 1 : length(THETA4)
        joint4_position_X = goal_position_X + (LINK_LENGTH(4) + LINK_LENGTH(5)) * cos(THETA4(i)) * cos(theta1(1));
        joint4_position_Y = goal_position_Y + (LINK_LENGTH(4) + LINK_LENGTH(5)) * cos(THETA4(i)) * sin(theta1(1));
        joint4_position_Z = goal_position_Z + (LINK_LENGTH(4) + LINK_LENGTH(5)) * sin(THETA4(i));
        %%%%%%%%%% calculate theta3
        theta3_temp = -(joint4_position_X^2 + joint4_position_Y^2 + (joint4_position_Z - LINK_LENGTH(1))^2 ...
                        - LINK_LENGTH(2)^2 - LINK_LENGTH(3)^2) / (2 * LINK_LENGTH(2) * LINK_LENGTH(3));
        theta3_temp = roundn(theta3_temp, log10(ALLOWABLE_ERROR)); 
        %%%%% if theta3_temp does not belong to this range, then
        %%%%% the goal position is not available.
        if abs(theta3_temp) <= 1 
            theta3(1) = atan2(sqrt(1 - theta3_temp^2), -theta3_temp);
            theta3(2) = atan2(-sqrt(1 - theta3_temp^2), -theta3_temp);              
            %%%%%%%%%%%% calculate theta2
            theta2_1 = atan2((joint4_position_Z - LINK_LENGTH(1)), sqrt(joint4_position_X^2 + joint4_position_Y^2));
            theta2_2 = atan2(LINK_LENGTH(3) * sin(theta3(1)), (LINK_LENGTH(2) + LINK_LENGTH(3) * cos(theta3(1))));
            theta2(2) = theta2_1 + theta2_2;
            theta2_2 = atan2(LINK_LENGTH(3) * sin(theta3(2)), (LINK_LENGTH(2) + LINK_LENGTH(3) * cos(theta3(2))));
            theta2(1) = theta2_1 + theta2_2;
            %%%%%% save results
            resultAngles(array_index + 0, 1 : 4) = [theta1(1)   theta2(1)   theta3(1) ...
                                                        (-pi + THETA4(i) - theta2(1) - theta3(1))];
            resultAngles(array_index + 1, 1 : 4) = [theta1(1)   theta2(2)   theta3(2) ...
                                                        (-pi + THETA4(i) - theta2(2) - theta3(2))];
            resultAngles(array_index + 2, 1 : 4) = [theta1(2) (pi - theta2(1)) -theta3(1) ...
                                                        -(-pi + THETA4(i) - theta2(1) - theta3(1))];
            resultAngles(array_index + 3, 1 : 4) = [theta1(2) (pi - theta2(2)) -theta3(2) ...
                                                        -(-pi + THETA4(i) - theta2(2) - theta3(2))];
            array_index = array_index + 4;
        end
    end
    
    display_count = 1;
    resultAngles(abs(resultAngles) <= ALLOWABLE_ERROR) = 0;
    resultAngles = roundn(resultAngles / deg2rad, log10(ALLOWABLE_ERROR));
    resultAngles = unique(resultAngles, 'rows');
    if array_index ~= 1
        for i = 1 : size(resultAngles, 1)
            if isCorrectAngles([resultAngles(i, :), angle_roll])
                disp(['solution ', num2str(display_count)])
                disp(['Theta1:', num2str(resultAngles(i, 1)), ' Theta2:', num2str(resultAngles(i, 2)), ...
                    ' Theta3:', num2str(resultAngles(i, 3)), ' Theta4:', num2str(resultAngles(i, 4)), ...
                    ' Theta5:', num2str(angle_roll)])
                display_count = display_count + 1;
            end
        end
    end
    if display_count == 1
        disp('There is no solution for this goal position !') 
    end
end

function result = isCorrectAngles(theta)

    global THETA1_MIN; global THETA1_MAX;
    global THETA2_MIN; global THETA2_MAX;
    global THETA3_MIN; global THETA3_MAX;
    global THETA4_MIN; global THETA4_MAX;
    global THETA5_MIN; global THETA5_MAX;
    
    if (theta(1) >= (THETA1_MIN)) && (theta(1) <= (THETA1_MAX)) ...
       && (theta(2) >= (THETA2_MIN)) && (theta(2) <= (THETA2_MAX)) ...
       && (theta(3) >= (THETA3_MIN)) && (theta(3) <= (THETA3_MAX)) ...
       && (theta(4) >= (THETA4_MIN)) && (theta(4) <= (THETA4_MAX)) ...
       && (theta(5) >= (THETA5_MIN)) && (theta(5) <= (THETA5_MAX))
        result = true;
    else
        result = false;
    end
end
