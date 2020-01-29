%% Inicjalizacja polaczenia z serwerem ROS

% rosinit

%% Inicjalizacja prawej oraz lewej reki

[rArm, rGoalMsg] = rosactionclient('r_arm_controller/joint_trajectory_action');
waitForServer(rArm);

rGoalMsg.Trajectory.JointNames = {'r_shoulder_pan_joint', ...
                                   'r_shoulder_lift_joint', ...
                                   'r_upper_arm_roll_joint', ...
                                   'r_elbow_flex_joint',...
                                   'r_forearm_roll_joint',...
                                   'r_wrist_flex_joint',...
                                   'r_wrist_roll_joint'};
                               
[lArm, lGoalMsg] = rosactionclient('l_arm_controller/joint_trajectory_action');
waitForServer(lArm);

lGoalMsg.Trajectory.JointNames = {'l_shoulder_pan_joint', ...
                                   'l_shoulder_lift_joint', ...
                                   'l_upper_arm_roll_joint', ...
                                   'l_elbow_flex_joint',...
                                   'l_forearm_roll_joint',...
                                   'l_wrist_flex_joint',...
                                   'l_wrist_roll_joint'};
           
% Przesuniecie obu rak do ustalonych pozycji w przestrzeni wzgledem pozycji
% robota
                               
% Punkt 2
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart = rosduration(2.0);

rGoalMsg.Trajectory.Points = tjPoint2;

sendGoalAndWait(rArm,rGoalMsg);

% Punkt 3
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [1.0 0.2 -0.1 -1.2 1.5 -0.3 0.5];
tjPoint3.Velocities = zeros(1,7);
tjPoint3.TimeFromStart = rosduration(2.0);

lGoalMsg.Trajectory.Points = tjPoint3;

sendGoalAndWait(lArm,lGoalMsg);

%% Tworzenie szkieletu robota w matlab

pr2 = exampleHelperWGPR2Kinect;

%% Pobranie informacji o statusie robota

jointSub = rossubscriber('joint_states');
jntState = receive(jointSub);

% Przypisanie pozycji przegubow do szkieletu robota

jntPos = exampleHelperJointMsgToStruct(pr2,jntState);

%% Wizualizacja konfiguracji robota

show(pr2,jntPos)

%% Utworzenie limitu ruchu torsu

torsoJoint = pr2.getBody('torso_lift_link').Joint; % Przypisanie torsu robota do pojedynczej zmiennej
idx = strcmp({jntPos.JointName}, torsoJoint.Name); % Porownanie nazw, proba znalezienia indeksu odpowiadajacego za tors robota
torsoJoint.HomePosition = jntPos(idx).JointPosition; % Okreslenie pozycji bazowej torsu na podstawie wczesniej pobranych pozycji
                                                     % robota
torsoJoint.PositionLimits = jntPos(idx).JointPosition + [-1e-3,1e-3]; % Ustalenie limitu ruchu torsu do pozycji bazowej z
                                                                      % marginesem bledu +- 0,001

%% Tworzenie odwrotnej kinematyki

ik = robotics.InverseKinematics('RigidBodyTree', pr2);

ik.SolverParameters.AllowRandomRestart = false; % Zabezpieczenie przed resetowaniem odwrotnej kinematyki

%% Określenie wag dla tolerancji kazdego komponentu wzgledem pozycji koncowej

weights = [0.25 0.25 0.25 1 1 1];
initialGuess = jntPos; % biezaca pozycja przypisana jako przyblizona

%% Okreslenie pozycji puszki w gazebo

% Nazwa punktu na bazie ktorego ruszamy manipulatorem

endEffectorName = 'r_gripper_tool_frame';

% Pozycja poczatkowa i oczekiwana dla puszki

TCanInitial = trvec2tform([0.766302, -0.085818, 0.779990]);
TCanFinal = trvec2tform([0.504421, -0.044294, 1.070150]);

%% Wzgledna transformata pomiedzy koncem manipulatora a puszka podczas chwytu
%  Funkcja trvec2tform przekonwertowuje wektor przesuniecia na transformate jednorodna 

TGraspToCan = trvec2tform([0,0,0.08])*eul2tform([pi/8,0,-pi]);

%% Określanie pozycji trajektorii manipulatora bazujacych na chwytaku

TGrasp = TCanInitial*TGraspToCan; % Oczekiwana pozycja chwytaka podczas lapania puszki
T1 = TGrasp*trvec2tform([0,0,-0.1]);
T2 = TGrasp*trvec2tform([0,0,-0.2]);
T3 = TCanFinal*TGraspToCan*trvec2tform([0,0,-0.2]);
TRelease = TCanFinal*TGraspToCan; % Oczekiwana pozycja chwytaka podczas wypuszczania puszki
T4 = T3*trvec2tform([-0.1,0,0]);

%% Sekwencja ruchu

motionTask = {'Release', T1, TGrasp, 'Grasp', T2, T3, TRelease, 'Release', T4};

%% Program wykonawczy

for i = 1: length(motionTask)
    
    if strcmp(motionTask{i}, 'Grasp')       % Przypisanie komend dla chwytaka: 'zlap'
        exampleHelperSendPR2GripperCommand('right',0.0,1000,true); 
        continue
    end
    %
    if strcmp(motionTask{i}, 'Release')     % Przypisanie komend dla chwytaka: 'pusc'
        exampleHelperSendPR2GripperCommand('right',0.1,-1,true);
        continue
    end  
    
    Tf = motionTask{i};
    % Pobranie aktualnych pozycji przegubów
    jntState = receive(jointSub);
    jntPos = exampleHelperJointMsgToStruct(pr2, jntState);
    
    % Obliczenia przeprowadzone w celu przeniesienia pozycji czlonow z
    % modelu do ramy robota, wykorzystujac jego konfiguracje
    T0 = getTransform(pr2, jntPos, endEffectorName); 
    
    % Interpolacja pomiedzy punktami posrednimi
    numWaypoints = 10;
    TWaypoints = exampleHelperSE3Trajectory(T0, Tf, numWaypoints); % Punkty posrednie dla chwytaka
    jntPosWaypoints = repmat(initialGuess, numWaypoints, 1); % Punkty posrednie dla przegubow
    
    rArmJointNames = rGoalMsg.Trajectory.JointNames;
    rArmJntPosWaypoints = zeros(numWaypoints, numel(rArmJointNames));
    
    % Obliczanie pozycji przegubow dla kazdej pozycji chwytaka przy
    % wykorzystaniu kinematyki odwrotnej
    for k = 1:numWaypoints
        jntPos = ik(endEffectorName, TWaypoints(:,:,k), weights, initialGuess);
        jntPosWaypoints(k, :) = jntPos;
        initialGuess = jntPos;
        
        % Pobranie pozycji prawego ramienia
        rArmJointPos = zeros(size(rArmJointNames));
        for n = 1:length(rArmJointNames)
            rn = rArmJointNames{n};
            idx = strcmp({jntPos.JointName}, rn);
            rArmJointPos(n) = jntPos(idx).JointPosition;
        end  
        rArmJntPosWaypoints(k,:) = rArmJointPos'; 
    end
    
    % Punkty czasu odpowiadajace poszczegolnym punktom posrednim
    timePoints = linspace(0,3,numWaypoints);
        
    % Numeryczna estymata predkosci przegubow na trajektorii
    h = diff(timePoints); h = h(1);
    jntTrajectoryPoints = arrayfun(@(~) rosmessage('trajectory_msgs/JointTrajectoryPoint'), zeros(1,numWaypoints)); 
    [~, rArmJntVelWaypoints] = gradient(rArmJntPosWaypoints, h);
    for m = 1:numWaypoints
        jntTrajectoryPoints(m).Positions = rArmJntPosWaypoints(m,:);
        jntTrajectoryPoints(m).Velocities = rArmJntVelWaypoints(m,:);
        jntTrajectoryPoints(m).TimeFromStart = rosduration(timePoints(m));
    end
    
    % Wizualizacja ruchu robota i chwytaka w matlabie
    hold on
    for j = 1:numWaypoints
        show(pr2, jntPosWaypoints(j,:),'PreservePlot', false);
        exampleHelperShowEndEffectorPos(TWaypoints(:,:,j));
        drawnow;
        pause(0.1);
    end
    
    % Wysylanie trajektori prawego ramienia do robota
    rGoalMsg.Trajectory.Points = jntTrajectoryPoints;
    sendGoalAndWait(rArm, rGoalMsg);

end

% Powrot do wyznaczonej pozycji

rGoalMsg.Trajectory.Points = tjPoint2;

sendGoalAndWait(rArm,rGoalMsg);

































































