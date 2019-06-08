%%%%%%%%%%%%%%%  YUAN JIALIN %%%%%%%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%%% December 2018 %%%%%%%%%%%%%
function screwTheory(goal_position_X, goal_position_Y, goal_position_Z, angle_pitch, angle_roll)

    global THETA1_MIN; global THETA1_MAX;
    global THETA2_MIN; global THETA2_MAX;
    global THETA3_MIN; global THETA3_MAX;
    global THETA4_MIN; global THETA4_MAX;
    global THETA5_MIN; global THETA5_MAX;
    
    THETA1_MIN = 0; THETA1_MAX = 180;
    THETA2_MIN = 0; THETA2_MAX = 180;
    THETA3_MIN = -180; THETA3_MAX = 0;
    THETA4_MIN = -90; THETA4_MAX = 90;
    THETA5_MIN = 0; THETA5_MAX = 180;

    LINK_LENGTH = [63.5, 152.4, 157.48, 34.29, 66.04];
%     LINK_LENGTH = [10, 10, 10, 10, 10];
    deg2rad = pi / 180;
    ALLOWABLE_ERROR = 0.00001;
    THETA4 = angle_pitch + 180;
    THETA4 = THETA4 * deg2rad;    
    
    w2 = [0; -1; 0];
    w3 = [0; -1; 0];
    Pa_0 = [LINK_LENGTH(2) + LINK_LENGTH(3); 0; LINK_LENGTH(1)];
    Pe_2 = [0; 0; LINK_LENGTH(1)];
    Pe_3 = [LINK_LENGTH(2); 0; LINK_LENGTH(1)];
    resultAngles = zeros(4,4);
    
    theta1 = [atan2(goal_position_Y, goal_position_X); rem(atan2(goal_position_Y, goal_position_X) + pi, 2 * pi)];
    for i = 1 : length(theta1)
        %%%%%%%%%% get joint4 position
        joint4_position_X = goal_position_X + (LINK_LENGTH(4) + LINK_LENGTH(5)) * cos(THETA4) * cos(theta1(i));
        joint4_position_Y = goal_position_Y + (LINK_LENGTH(4) + LINK_LENGTH(5)) * cos(THETA4) * sin(theta1(i));
        joint4_position_Z = goal_position_Z + (LINK_LENGTH(4) + LINK_LENGTH(5)) * sin(THETA4);

        e1 = [1 0 0; 0 1 0; 0 0 1] + [0 1 0;-1 0 0;0 0 0]*sin(theta1(i)) + ...
            [0 1 0;-1 0 0;0 0 0] * [0 1 0;-1 0 0;0 0 0] * (1 - cos(theta1(i)));
        Pa_dot = [e1 ([1 0 0;0 1 0;0 0 1] - e1) * [0;0;0]; 0 0 0 1] * [joint4_position_X;joint4_position_Y;joint4_position_Z;0];
        u = Pa_0 - Pe_3;
        v = Pa_dot(1:3) - Pe_2;
        radius_23 = Pe_2 - Pe_3;
        radius_32 = Pe_3 - Pe_2;

        theta02 = atan2(dot(w2, cross(radius_32, v)), dot(radius_32, v));
        theta03 = atan2(dot(w3, cross(u, radius_23)), dot(u, radius_23));

        alpha = acos((norm(v)^2 + norm(radius_32)^2 - norm(u)^2) / (2 * norm(v) * norm(radius_32)));
        beta = acos((norm(u)^2 + norm(radius_32)^2 - norm(v)^2) / (2 * norm(u) * norm(radius_32)));
        %%%%%%%%%% this IF-ELSE part is used for eliminating the error of float data 
        if abs(imag(alpha) + imag(beta)) < ALLOWABLE_ERROR 
            alpha = real(alpha);
            beta = real(beta);
        end
        
        if isreal(alpha) && isreal(beta)
            
            theta2 = [alpha + theta02; alpha - theta02];
            theta3 = [beta - theta03; beta + theta03];
            if i == 1
                theta4 = [THETA4 - theta2(1) - theta3(1) - pi; THETA4 - theta2(2) - theta3(2) - pi];
            else
                theta4 = [-THETA4 - theta2(1) - theta3(1) + 2 * pi; -THETA4 - theta2(2) - theta3(2) + 2 * pi];
            end
            resultAngles(i * 2 - 1, 2 : 4) = [theta2(1) theta3(1) theta4(1)];
            resultAngles(i * 2    , 2 : 4) = [theta2(2) theta3(2) theta4(2)];

        else
            resultAngles(i * 2 - 1, 2 : 4) = [pi pi pi] * pi;
            resultAngles(i * 2    , 2 : 4) = [pi pi pi] * pi; 
        end
    end
    
    resultAngles(:,1) = [theta1(1); theta1(1); theta1(2); theta1(2)];
    display_count = 1;
    %%%%%%%%%% this part is used for eliminating the error of float data 
    resultAngles(abs(resultAngles) <= ALLOWABLE_ERROR) = 0;
    resultAngles = resultAngles / deg2rad;
    resultAngles = roundn(resultAngles, log10(ALLOWABLE_ERROR));
    resultAngles = unique(resultAngles, 'rows');
    %%%%%%%%%% judge which angles set is correct and output it
    for i = 1 : size(resultAngles, 1)
        if isCorrectAngles([resultAngles(i,:), angle_roll])
            disp(['solution ', num2str(display_count)])
            disp(['Theta1:', num2str(resultAngles(i, 1)), ' Theta2:', num2str(resultAngles(i, 2)), ...
                ' Theta3:', num2str(resultAngles(i, 3)), ' Theta4:', num2str(resultAngles(i, 4)), ...
                ' Theta5:', num2str(angle_roll)])
            display_count = display_count + 1;
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
