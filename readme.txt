%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
STUDENT NAME:    Yuan Jialin; Lu Qian

P.S. Regarding the report file, if the docx cannot work, there is also a pdf version.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RELEVANT MATLAB CODE (listed according to the report structure tree):

#### [PART I] 

&&&& Task A

**** 1) Forward Kinematics-DH
>>>> FK_get_T0e.m
This is a function. To run it, you need to input five parameters of each joint (joint_variable_1 ,joint_variable_2 ,joint_variable_3 ,joint_variable_4 ,joint_variable_5);

**** 2) Workspace
>>>> part1_a_workspace.m
----This is a script. You can change the parameters directly If you want to run with other parameters.

**** 3) Inverse kinematics
>>>> inverseKinematic.m 
----This is a function. To run it, you need to input five parameters of the end-effector (goal_position_X, goal_position_Y, goal_position_Z, angle_pitch, angle_roll);


&&&& Task B
**** 1) FK & IK test
>>>> part1_b_freemotion.m
----This is a script using FK_get_T0e.m and IK.m . You can change the parameters directly If you want to run with other parameters.

**** 2) Free motion path-planning
>>>> part1_b_freemotion.m
----This is a script using FK_get_T0e.m and IK.m . You can change the parameters directly If you want to run with other parameters.

**** 3) Straight line trajectory planning
>>>> part1_b_straightline.m
----This is a script using FK_get_T0e.m and IK.m . You can change the parameters directly If you want to run with other parameters.

**** 4) Obstacle avoidance planning
>>>> part1_b_obstacleavoid.m
----This is a script using FK_get_T0e.m and IK.m . You can change the parameters directly If you want to run with other parameters.



#### [PART II]
&&&& 1) Inverse kinematics
>>>> part2_1_inversekinematics.m
----This is a script. You can change the parameters directly If you want to run with other parameters.

&&&& 2) Workspace
>>>> part2_2_drawWorkspaceIK.m
----This is a script. You can change the parameters directly If you want to run with other parameters.




#### [PART III]

&&&&Screw theory
>>>> screwTheory.m
----This is a function. To run it, you need to input five parameters of the end-effector (goal_position_X, goal_position_Y, goal_position_Z, angle_pitch, angle_roll);

&&&& Jacobian-based IK solution of lynxmotion arm
>>>> J_number.m
----This is a function. To run it, you need to input four parameters of the end-effector (goal_position_X, goal_position_Y, goal_position_Z, angle_pitch);
    