classdef Lab_assignment_2 < handle

    properties
        AppleInitial;
        rob1Apples;
        rob2Apples;
        rob1;
        rob2;
        rob1Grip1;
        rob1Grip2;
        rob1Grip3;
        rob2Grip1;
        rob2Grip2;
        rob2Grip3;
        current_item;
        items_pos;
        cube;
        Apples;
        robot;
        Truck;
        rmrc_step;
        rob1_link_points;
        rob2_link_points;
        EmergencyFlag;
    end

    properties(Constant)
        trsteps = 50;
        gsteps = 20;
        gripperoffset = 0.09;
        gripOpen = [0,2*pi/9,pi/9];
        gripClose = [0,5*pi/12,pi/12];

        % initialGuessQRob1 = [-0.7849   -1.7363    0.0001    0.1327   -1.6231    0.7849];
        rob1PickInitialGuess = [-1.5010, -0.4014, 0.4712, 3.0718, -1.5533, 0];
        rob1PlaceInitialGuess = [-4.2672, -0.0367, 1.2988, -2.7664, 4.7124, -2.6964];
        rob2PickInitialGuess = [-0.1920, 0.7369, 0.5042, 0, 0, 0];
        rob2PlaceInitialGuess = [-1.9005, 1.6678, 1.4350, 0, 0, 0];

        apple_pos1 = [deg2rad(43.2), deg2rad(-43.2), deg2rad(-14.4), deg2rad(28.8), deg2rad(43.2), deg2rad(28.8)];  % From teach
        test_single_pos = transl(0.4,0,0.2) * trotx(-pi);

        test_multiple_pos = {transl(0.4,0,0.3)*trotx(-pi), ...
                            transl(-0.4,0,0.3)*trotx(-pi), ...
                            transl(0.4,0.1,0.3)*trotx(-pi), ...
                            transl(-0.4,0.1,0.3)*trotx(-pi), ...
                            transl(-0.2,0.2,0.5)*trotx(-pi), ...
                            transl(0,0,0.6)*trotx(-pi)};
        AppleFinal = {transl(1, -0.08, 0.36)*trotx(pi), ...
                      transl(1, 0, 0.36)*trotx(pi), ...
                      transl(1, 0.08, 0.36)*trotx(pi), ...
                      transl(1.08, -0.08, 0.36)*trotx(pi), ...
                      transl(1.08, 0, 0.36)*trotx(pi), ...
                      transl(1.08, 0.08, 0.36)*trotx(pi), ...
                      transl(1.16, -0.08, 0.36)*trotx(pi)};
        PlaceApples = [1, -0.08, 0.41;
                       1, 0, 0.41;
                       1, 0.08, 0.41;
                       1.08, -0.08, 0.41;
                       1.08, 0, 0.41;
                       1.08, 0.08, 0.41;
                       1.16, -0.08, 0.41];

        ur3_link_sizes = {[0.05,0.05,0.1519], ...
                        [0.24365,0.05,0.05], ...
                        [0.21325,0.05,0.05], ...
                        [0.05,0.05,0.11235], ...
                        [0.05,0.05,0.08535], ...
                        [0.05,0.05,0.0819], ...
                        [0.05,0.05,0.05]};

        ur5_link_sizes_for_collisions = {[0.05,0.05,0.089159/2.5], ...
                                        [-0.425/2,0.05,0.05], ...
                                        [-0.39225/2,0.05,0.05], ...
                                        [0.05,0.05,0.10915/2], ...
                                        [0.05,0.05,0.09465/2], ...
                                        [0.05,0.05,0.0823/2], ...
                                        [0.05,0.05,0.05]};   

        ur5_link_sizes = {[0.05,0.05,0.089159], ...
                        [-0.425,0.05,0.05], ...
                        [-0.39225,0.05,0.05], ...
                        [0.05,0.05,0.10915], ...
                        [0.05,0.05,0.09465], ...
                        [0.05,0.05,0.0823], ...
                        [0.05,0.05,0.05]};   

        
        % % Cell array for the links of the UR3 with q as all zeros
        % UR3_links = {eye(4), ...                                                        % Base transform
        %             trotz(0) * transl(0,0,0.1519) * transl(0,0,0) * trotx(pi/2), ...    % joint0To1
        %             trotz(0) * transl(0,0,0) * transl(-0.24365,0,0) * trotx(0), ...  % joint1To2
        %             trotz(0) * transl(0,0,0) * transl(-0.21325,0,0) * trotx(0), ...  % joint2To3
        %             trotz(0) * transl(0,0,0.11235) * transl(0,0,0) * trotx(pi/2), ...% joint3To4
        %             trotz(0) * transl(0,0,0.08535) * transl(0,0,0) * trotx(-pi/2), ...% joint4To5
        %             trotz(0) * transl(0,0,0.0819) * transl(0,0,0) * trotx(0), ...    % joint5To6
        %             eye(4)};                                                            % Tool Transform
    end

    methods
        %% Constructor
        function self = Lab_assignment_2(robot1, robot2, Apples)
            % Create the environment within the constructor
            if nargin > 0
                self.rob1 = robot1;
                self.rob2 = robot2;     
                self.AppleInitial = Apples;

            else
                self.rob1 = UR5;
                self.rob2 = KUKAKR;
                
                self.rmrc_step = 0;
                self.rob1_link_points = cell(1,6);
                self.rob2_link_points = cell(1,6);
                self.items_pos = self.test_multiple_pos;
                self.EmergencyFlag = false;
                self.AppleInitial = {transl(0, 1, 1), ...
                                     transl(0.3, 1.3, 0.7), ...
                                     transl(0.5, 1.2, 1.1), ...
                                     transl(0.7, 1.2, 0.9), ...
                                     transl(0.4, 1.1, 1.5), ...
                                     transl(0.75, 1.2, 1.75), ...
                                     transl(1.4, 1.1, 1.25)};
            end

            hold on
            % self.cube = PlaceObject("BasicObstacle.ply",[0.5,0,0.2]);
            self.setup_environment();

            self.rob1Grip1 = Gripper(self.rob1.model.fkine(self.rob1.model.getpos()).T);
            self.rob1Grip2 = Gripper(self.rob1.model.fkine(self.rob1.model.getpos()).T * trotz(2*pi/3));
            self.rob1Grip3 = Gripper(self.rob1.model.fkine(self.rob1.model.getpos()).T * trotz(4*pi/3));
            self.rob2Grip1 = Gripper(self.rob2.model.fkine(self.rob2.model.getpos()).T);
            self.rob2Grip2 = Gripper(self.rob2.model.fkine(self.rob2.model.getpos()).T * trotz(2*pi/3));
            self.rob2Grip3 = Gripper(self.rob2.model.fkine(self.rob2.model.getpos()).T * trotz(4*pi/3));

            axis([-2 2 -2 2 0 2.2]);
            self.assignApples(1.2);
        end


        %% Assign which apples will be picked by which robots
        function assignApples(self, threshold)
    % Check if the emergency flag is set to false
    if ~self.EmergencyFlag
        s = size(self.AppleInitial);
        index1 = 1;
        index2 = 1;
        for i = 1:s(2)
            if ~self.EmergencyFlag
                if self.AppleInitial{1,i}(3,4) < threshold
                    self.rob1Apples{1,index1} = self.AppleInitial{1,i};
                    index1 = index1 + 1;
                elseif self.AppleInitial{1,i}(3,4) > threshold
                    self.rob2Apples{1,index2} = self.AppleInitial{1,i};
                    index2 = index2 + 1;
                end
            else
                disp('Estop triggered')
                input('Once work environent is safe press ENTER to resume operation')
                self.EmergencyFlag = false;
            end

        end
    else
        % If the emergencyFlag is true we deal with that here
        
        disp('Emergency flag is set. function was in assignApples.');
        input('Once work environent is safe press ENTER to resume operation')
        self.EmergencyFlag = false;

    end
end

        %% Change the Transform/pose relative to robot base into q values
        function q1 = transformation2Q_rob1(self, transform, InitialGuess)
    if ~self.EmergencyFlag  % Check if emergency flag is not set
        if numel(InitialGuess) ~= 6
            q1 = self.rob1.model.ikcon(transform);
        else
            q1 = self.rob1.model.ikcon(transform,InitialGuess);
        end
    else
        disp('Emergency stop engaged. Program was in transformation2Q_rob1.');
        input('Once work environent is safe press ENTER to resume operation')
        self.EmergencyFlag = false;

        %q1 = [];  Do we need to zero q1?
    end
end
        function q2 = transformation2Q_rob2(self, transform, InitialGuess)
    if ~self.EmergencyFlag  % Check if emergency flag is not set
        if numel(InitialGuess) ~= 6
            q2 = self.rob2.model.ikcon(transform);
        else
            q2 = self.rob2.model.ikcon(transform,InitialGuess);
        end
    else
        disp('Emergency stop engaged. Pragram was in transformation2Q_rob2.');
        input('Once work environent is safe press ENTER to resume operation')
        self.EmergencyFlag = false;
        %q2 = [];  
    end
end

        %% Change the robot q values into an end effector transform
        function transform1 = q2Transform_rob1(self, q)
            transform1 = self.rob1.model.fkine(q);
        end

        function transform2 = q2Transform_rob2(self, q)
            transform2 = self.rob2.model.fkine(q);
        end


        %% Transform function, can be used to move the robots from one orientation to the next
        function transform_interpolation(self, start_q_rob1, end_q_rob1, start_q_rob2, end_q_rob2, gripper)

            % robot.model.getpos
            % UR3 = UR3_control.robot;
            
            r1Traj = jtraj(start_q_rob1,end_q_rob1,self.trsteps);
            r2Traj = jtraj(start_q_rob2,end_q_rob2,self.trsteps);

            for i = 1:self.trsteps
                if ~self.EmergencyFlag
                    % r2.model.fkine(UR3.model.getpos)
                    self.rob1.model.animate(r1Traj(i,:));
                    self.rob2.model.animate(r2Traj(i,:));
                    % self.update_gripper_pos();
                    self.moveGripper1(gripper);
                    self.moveGripper2(gripper);
                    % collision_check = self.checkCollisions();
                    % if collision_check == 1
                    %     disp("collision detected!!!")
                    %     break
                    % end
                else 
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end

                drawnow()
            end
            
        end

        %% Robotic control via Resolved Motion Rate Control
        function rmrc(self, end_eff_start_rob1, end_eff_end_rob1, end_eff_start_rob2, end_eff_end_rob2)
            t = 10;             % Total time (s)
            deltaT = 0.05;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error

            % 1.3) Set up trajectory, initial pose
            RPY_start = tr2rpy(end_eff_start_rob1);
            RPY_end = tr2rpy(end_eff_end_rob1);
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                if~self.EmergencyFlag
                x(1,i) = (1-s(i))*0.35+s(i)*0.35;%(1-s(i))*end_eff_start_rob1(1,4) + s(i)*end_eff_end_rob1(1,4);%(1-s(i))*0.35 + s(i)*0.35; % Points in x
                x(2,i) = (1-s(i))*-0.5+s(i)*0.5;%(1-s(i))*end_eff_start_rob1(2,4) + s(i)*end_eff_end_rob1(2,4); % Points in y
                x(3,i) = 0.2;%(1-s(i))*end_eff_start_rob1(3,4) + s(i)*end_eff_end_rob1(3,4);%0.25;% + 0.2*sin(i*delta); % Points in z
                theta(1,i) = 0;%RPY_start(1,1);                 % Roll angle 
                theta(2,i) = 5*pi/9;%RPY_start(1,2);%5*pi/9;            % Pitch angle
                theta(3,i) = 0;%RPY_start(1,3);%0;                 % Yaw angle
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end
                
            end
            % rotation = end_eff_start_rob1(1:3,1:3);

            % Transform becomes imaginary here
            % T1 = transl(x(1,1),x(1,2),x(1,3))*troty(theta(2,1));%trotx(theta(1,1))*troty(theta(2,1))*trotz(theta(3,1));%troty(theta(2,1));
            T1 = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix(1,:) = self.rob1.model.ikcon(T1,q0);                                            % Solve joint angles to achieve first waypoint


            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                if~self.EmergencyFlag
                    % UPDATE: fkine function now returns an SE3 object. To obtain the 
                    % Transform Matrix, access the variable in the object 'T' with '.T'.
                    T = self.rob1.model.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
                    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                    S = Rdot*Ra';                                                            % Skew symmetric!
                    linear_velocity = (1/deltaT)*deltaX;
                    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    % deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                    J = self.rob1.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                    m(i) = sqrt(det(J*J'));
                    if m(i) < epsilon  % If manipulability is less than given threshold
                        lambda = (1 - m(i)/epsilon)*5E-2;
                    else
                        lambda = 0;
                    end
                    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                    for j = 1:6                                                             % Loop through joints 1 to 6
                        if~self.EmergencyFlag
                        if qMatrix(i,j) + deltaT*qdot(i,j) < self.rob1.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                            qdot(i,j) = 0; % Stop the motor
                        elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.rob1.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                            qdot(i,j) = 0; % Stop the motor
                        end
                        else
                            disp('Estop Triggered')
                            input('Once work environent is safe press ENTER to resume operation')
                            self.EmergencyFlag = false;
                        end
                    end
                    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                    % angleError(:,i) = deltaTheta;                                           % For plotting
                    % keyboard
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end
            end
            
            % 1.5) Plot the results
            tic
            
            % plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            for i = 1:steps
                if~self.EmergencyFlag
                self.rob1.model.animate(qMatrix(i,:));
                hold on
                end_eff = self.rob1.model.fkine(qMatrix(i,:));
                end_eff = end_eff.T;
                plot3(end_eff(1,4),end_eff(2,4),end_eff(3,4),'r.');
                drawnow();
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end
            end
            
            disp(['Plot took ', num2str(toc), 'seconds'])
        end

        

        %% Function used to begin picking up apples autonomously
        function pickApples(self,fig)
            figure(fig);
            hold on
            rob1applessize = size(self.rob1Apples);
            rob2applessize = size(self.rob2Apples);
            num_items = max(rob1applessize(2),rob2applessize(2));
            % num_items = size(self.test_multiple_pos);
% Idea for how to have both robots picking up apples at the same time:
% Have one big matrix for the positions of the apples and in that matrix
% also encode a boolean which tells whether it is currently being picked or
% not. So the cell array would look like this:
%   [transform for item position, true;
%   transform for item position, true;
%   transform for item position, false;
%   transform for item position, false]

            for i = 1:num_items
                if~self.EmergencyFlag
                self.current_item = i;
                for pick_place = 1:2 % pick apple = 1, place apple = 2
                    currentFigure = gcf;
                    figureName = get(currentFigure, 'Name');
                    disp(figureName);

                    % Get current position and the q at that position
                    rob1qPos1 = self.rob1.model.getpos();
                    rob2qPos1 = self.rob2.model.getpos();

                    % disp(["The current apple is number: ",i]);
                    % disp("The current apple being picked has the transform: ");
                    if pick_place == 1
                        if i > rob1applessize(2)
                            rob1qPos2 = [-90 -90 0 0 0 0];
                        else
                            gripper_pos = self.rob1Apples{i} * trotx(-pi/2) * transl(0, 0, -0.05);
                            % disp(self.AppleInitial{i});
                            rob1qPos2 = self.transformation2Q_rob1(gripper_pos,self.rob1PickInitialGuess);
                        end

                        if i > rob2applessize(2)
                            rob2qPos2 = [0 0 0 0 0 0];
                        else
                            gripper_pos = self.rob2Apples{i} * trotx(-pi/2) * transl(0, 0, -0.05);
                            rob2qPos2 = self.transformation2Q_rob2(gripper_pos,self.rob2PickInitialGuess);
                        end
                    elseif pick_place == 2
                        if i > rob1applessize(2)
                            rob1qPos2 = [-90 -90 0 0 0 0];
                        else
                            gripper_pos = self.AppleFinal{i} * transl(0, 0, -0.13);
                            % disp(self.AppleFinal{i});
                            rob1qPos2 = self.transformation2Q_rob1(gripper_pos,self.rob1PlaceInitialGuess);
                            % qPos2 = self.rob1.model.ikcon(gripper_pos); %,[-4.2672,-0.0367,1.2988,-2.7664,4.7124,-2.6964]);
                        end

                        if i > rob2applessize(2)
                            rob2qPos2 = [0 0 0 0 0 0];
                        else
                            gripper_pos = self.AppleFinal{end+1-i} * transl(0, 0, -0.13);
                            rob2qPos2 = self.transformation2Q_rob2(gripper_pos,self.rob2PlaceInitialGuess);
                        end
                    end
                    % Gripper positioned z + 0.09 of brick.
                    % gripper_pos = self.items_pos{i};
                    % gripper_pos(3,4) = gripper_pos(3,4) + self.gripperoffset;
                    % Calc q at the end position
                    % qPos2 = self.transformation2Q_rob1(gripper_pos);
                
                    % Open gripper and transform between current position and
                    % starting brick position
                    % self.open_gripper();
                    self.transform_interpolation(rob1qPos1,rob1qPos2,rob2qPos1,rob2qPos2,pick_place);

                    self.opencloseGripper1(pick_place);
                    self.opencloseGripper2(pick_place);

                end

                figure(fig);
                if i <= rob1applessize(2)
                    delete(self.Apples(i,1));
                    self.Apples(i,1) = PlaceObject('NewApple.ply',self.PlaceApples(i,:));
                end
                if i <= rob2applessize(2)
                    delete(self.Apples(end+1-i,1));
                    self.Apples(end+1-i,1) = PlaceObject('NewApple.ply',self.PlaceApples(end+1-i,:));
                end
                drawnow;
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end
            end

            hold off
        end


        %% Function used to check for collisions and return a true/false if the robot will collide with anything, including the other robot
        function collision = checkCollisions(self)
            
            collision = 0;
            EllipsoidCenterPoints = cell(1,7);
            transforms = cell(1,7);
            midpoints = cell(1,7);
            NumberOfLinks = size(self.rob1.model.links);
            % disp(["Number of links on the robot: ", NumberOfLinks]);
            current_robq = self.rob1.model.getpos;

            % Initial ellipsoid centerpoint

            for i = 1:NumberOfLinks(1,2)+1
                if~self.EmergencyFlag
                [X,Y,Z] = ellipsoid(0,0,0,self.ur5_link_sizes_for_collisions{i}(1),self.ur5_link_sizes_for_collisions{i}(2),self.ur5_link_sizes_for_collisions{i}(3));
                EllipsoidPoints = [X(:),Y(:),Z(:)];

                transforms{i} = self.manual_fkine_rob1(current_robq,i);
                EllipsoidCenterPoints{i} = transforms{i}(1:3,4);
                % Link 1 midpoint
                if i == 1
                    % The link is translated and then rotated so we need to
                    % determine the transform without the rotation
                    midpoints{i} = transforms{i}*trotx(-pi/2);
                    base = self.rob1.model.base;
                    base = base.T; 
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + base(1:3,4))/2;
 
                % Link 4 midpoint
                elseif i == 4
                    midpoints{i} = transforms{i}*trotx(-pi/2);
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;

                % Link 5 midpoint
                elseif i == 5
                    midpoints{i} = transforms{i}*trotx(pi/2);
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                
                % All other link midpoints
                else
                    midpoints{i} = transforms{i};
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                end

                hold on
                EllipsoidPointsAndOnes = [midpoints{i} * [EllipsoidPoints,ones(size(EllipsoidPoints,1),1)]']';
                updatedEllipsoidPoints = EllipsoidPointsAndOnes(:,1:3);
                self.rob1_link_points{i} = updatedEllipsoidPoints;
                % plot3(updatedEllipsoidPoints(:,1), EllipsoidPointsAndOnes(:,2), EllipsoidPointsAndOnes(:,3));
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end 
            end
            
            
            verts = [get(self.Truck,'Vertices'), ones(size(get(self.Truck,'Vertices'),1), 1)];
            verts(:,4) = [];

            % plot3(verts(:,1),verts(:,2),verts(:,3));
            % plot3(0.3157, 0.3872, 1.0487);
            % disp(verts);

            tic

            
            
            for i = 1:NumberOfLinks(1,2)+1
                % Old detection method
                % close_verts = cell(0,0);
                % 
                % d = ((verts(:,1)-midpoints{i}(1,4))/self.ur5_link_sizes_for_collisions{i}(1)).^2 ...
                %   + ((verts(:,2)-midpoints{i}(2,4))/self.ur5_link_sizes_for_collisions{i}(2)).^2 ...
                %   + ((verts(:,3)-midpoints{i}(3,4))/self.ur5_link_sizes_for_collisions{i}(3)).^2;
                % 
                % % disp(d);
                % for j = 1:size(d)
                %     disp(d(j));
                %     if d(j) < 1
                %         % close_verts{end+1} = verts(j);
                %         disp("########");
                % 
                %     end
                %     keyboard
                % end

                % close_verts = [];
                % d = ((verts(:,1)-midpoints{i}(1,4)).^2 + (verts(:,2)-midpoints{i}(2,4)).^2 + (verts(:,3)-midpoints{i}(3,4)).^2);
                % disp(d);
                % disp("The max radii in the link is: ")
                % disp(max(self.ur5_link_sizes_for_collisions{i}));

                % for j = 1:(size(verts))
                %     if abs(d*2) < max(abs(self.ur5_link_sizes_for_collisions{i}))
                %         close_verts = [close_verts;verts(j,:)];
                %     end
                % end
                % disp("The close vertices are: ");
                % disp(close_verts);
                


                invT_ = inv(midpoints{i});

                for k = 1:size(verts)
                    if~self.EmergencyFlag
                    vertex_transform = midpoints{i};
                    vertex_transform(1:3,4) = [verts(k,1:3)]';  % Keep rotation from links but change the position to the vertex of ply file
                    % vertex_transform = transl(verts(k,1),verts(k,2),verts(k,3));
                    link_to_vertex = invT_*vertex_transform;

                    % disp([verts(k,1),verts(k,2),verts(k,3)]);
                    % disp([midpoints{i}(1,4),midpoints{i}(2,4),midpoints{i}(3,4)]);
                    % disp(vertex_transform);

                    if (abs(link_to_vertex(1,4)) <= abs(self.ur5_link_sizes_for_collisions{i}(1))) && ...
                        (abs(link_to_vertex(2,4)) <= abs(self.ur5_link_sizes_for_collisions{i}(2))) && ...
                        (abs(link_to_vertex(3,4)) <= abs(self.ur5_link_sizes_for_collisions{i}(3)))

                        % disp([abs(link_to_vertex(1,4)), abs(self.ur5_link_sizes_for_collisions{i}(1))]);
                        % disp([abs(link_to_vertex(2,4)), abs(self.ur5_link_sizes_for_collisions{i}(2))]);
                        % disp([abs(link_to_vertex(3,4)), abs(self.ur5_link_sizes_for_collisions{i}(3))]);

                        disp("intersection detected");
                        collision = 1;
                        disp(["Intersection link number: ", i]);
                        % disp(k);
                        disp("Collision detected at: ");
                        disp(["x:", verts(k,1), "y: ", verts(k,2), "z: ", verts(k,3)]);

                        [X,Y,Z] = ellipsoid(0,0,0,self.ur5_link_sizes_for_collisions{i}(1),self.ur5_link_sizes_for_collisions{i}(2),self.ur5_link_sizes_for_collisions{i}(3));
                        EllipsoidPoints = [X(:),Y(:),Z(:)];
                        EllipsoidPointsAndOnes = [midpoints{i} * [EllipsoidPoints,ones(size(EllipsoidPoints,1),1)]']';
                        updatedEllipsoidPoints = EllipsoidPointsAndOnes(:,1:3);
                        plot3(updatedEllipsoidPoints(:,1), EllipsoidPointsAndOnes(:,2), EllipsoidPointsAndOnes(:,3));

                        % disp([midpoints{i}(1,4),midpoints{i}(2,4),midpoints{i}(3,4)]);
                        % disp(vertex_transform);
                        % disp(T);
                        % disp(link_to_vertex);
                        % disp([self.ur5_link_sizes_for_collisions{i}(1),self.ur5_link_sizes_for_collisions{i}(2),self.ur5_link_sizes_for_collisions{i}(3)]);

                        break
                        % ellipsoid(midpoints{i}(1,4),midpoints{i}(2,4),midpoints{i}(3,4),self.ur5_link_sizes_for_collisions{i}(1),self.ur5_link_sizes_for_collisions{i}(1),self.ur5_link_sizes_for_collisions{i}(1));
                    end
                    else
                        disp('Estop Triggered')
                        input('Once work environent is safe press ENTER to resume operation')
                        self.EmergencyFlag = false;
                    end
                end


            end
            disp(["Time to finish the for loop: " toc]);

            

            % So the plan is (Check Lab6Solution question 2 and Lab6_collision_detection):
                % - Create ellipsoid around links, figure out the midpoint of the each link (radii values
                % are hardcoded for the robot, in the DH parameters it's
                % x,y and z

                % - Get the vertices of the ply file

                % - Get the relative transform between each link and each
                % point/vertex in the ply files.

                % - If the x,y and z positions of the vertex relative to
                % the midpoint of the link are within the x,y and z link
                % sizes return true, if the relative x,y and z are not
                % within the link size then return false.

                % - If false then keep moving, if true then stop movement. Might need
                % to actively avoid certain zones, gotta do stopping first.

            
        end


        function Transform = manual_fkine_rob1(self,q,return_link)

            if return_link > 7  % 6 degrees of freedom and a tool transform
                disp("The transform for the link number cannot be returned as it exceeds the UR5 robot's links");
                return
            end
            linksArray = {self.rob1.model.base, ...
                        trotz(q(1)) * transl(0,0,0.089159) * transl(0,0,0) * trotx(pi/2), ...
                        trotz(q(2)) * transl(0,0,0) * transl(-0.425,0,0) * trotx(0), ...
                        trotz(q(3)) * transl(0,0,0) * transl(-0.39225,0,0) * trotx(0), ...
                        trotz(q(4)) * transl(0,0,0.10915) * transl(0,0,0) * trotx(pi/2), ...
                        trotz(q(5)) * transl(0,0,0.09465) * transl(0,0,0) * trotx(-pi/2), ...
                        trotz(q(6)) * transl(0,0,0.0823) * transl(0,0,0) * trotx(0), ...
                        eye(4)};
            % linksArray = {base_tr, joint1To2, joint2To3, joint3To4, joint4To5, joint5To6, joint6To7, tool_tr};
            

            Transform = linksArray{1}.T;
            for i = 2:return_link+1
                if~self.EmergencyFlag
                Transform = Transform * linksArray{i};
                else
                    disp('Estop Triggered')
                    input('Once work environent is safe press ENTER to resume operation')
                    self.EmergencyFlag = false;
                end
            end

            % End effector Transform = base_tr*joint0To1*joint1To2*joint2To3*joint3To4*joint4To5*joint5To6*joint6To7*tool_tr; 
        end


        function plot_robot_no_ply(self)
            L1 = Link('d',0.089159,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0,'a',-0.425,'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L3 = Link('d',0,'a',-0.39225,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L4 = Link('d',0.10915,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L5 = Link('d',0.09465,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0.0823,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);   
             
            self.robot = SerialLink([L1,L2,L3,L4,L5,L6],'name','random_robot');

            self.robot.plot(zeros(1,6));
        end

    %% Gripper
    function moveGripper1(self, openclose)
        self.rob1Grip1.model.base = self.rob1.model.fkine(self.rob1.model.getpos()).T;
        self.rob1Grip2.model.base = self.rob1.model.fkine(self.rob1.model.getpos()).T * trotz(2*pi/3);
        self.rob1Grip3.model.base = self.rob1.model.fkine(self.rob1.model.getpos()).T * trotz(4*pi/3);

        if openclose == 1
            self.rob1Grip1.model.animate(self.gripOpen);
            self.rob1Grip2.model.animate(self.gripOpen);
            self.rob1Grip3.model.animate(self.gripOpen);
        elseif openclose == 2
            self.rob1Grip1.model.animate(self.gripClose);
            self.rob1Grip2.model.animate(self.gripClose);
            self.rob1Grip3.model.animate(self.gripClose);
        end
    end

    function moveGripper2(self, openclose)
        self.rob2Grip1.model.base = self.rob2.model.fkine(self.rob2.model.getpos()).T;
        self.rob2Grip2.model.base = self.rob2.model.fkine(self.rob2.model.getpos()).T * trotz(2*pi/3);
        self.rob2Grip3.model.base = self.rob2.model.fkine(self.rob2.model.getpos()).T * trotz(4*pi/3);

        if openclose == 1
            self.rob2Grip1.model.animate(self.gripOpen);
            self.rob2Grip2.model.animate(self.gripOpen);
            self.rob2Grip3.model.animate(self.gripOpen);
        elseif openclose == 2
            self.rob2Grip1.model.animate(self.gripClose);
            self.rob2Grip2.model.animate(self.gripClose);
            self.rob2Grip3.model.animate(self.gripClose);
        end
    end

    function opencloseGripper1(self, openclose)
        if openclose == 1
            gripperTraj = jtraj(self.gripOpen,self.gripClose,self.gsteps);
        elseif openclose == 2
            gripperTraj = jtraj(self.gripClose,self.gripOpen,self.gsteps);
        end

        for j = 1:self.gsteps
            if~self.EmergencyFlag
            self.rob1Grip1.model.animate(gripperTraj(j,:));
            self.rob1Grip2.model.animate(gripperTraj(j,:));
            self.rob1Grip3.model.animate(gripperTraj(j,:));
            drawnow;
            else
                disp('Estop Triggered')
                input('Once work environent is safe press ENTER to resume operation')
                self.EmergencyFlag = false;
            end
        end
    end

    function opencloseGripper2(self, openclose)
        if openclose == 1
            gripperTraj = jtraj(self.gripOpen,self.gripClose,self.gsteps);
        elseif openclose == 2
            gripperTraj = jtraj(self.gripClose,self.gripOpen,self.gsteps);
        end

        for j = 1:self.gsteps
            if~self.EmergencyFlag
            self.rob2Grip1.model.animate(gripperTraj(j,:));
            self.rob2Grip2.model.animate(gripperTraj(j,:));
            self.rob2Grip3.model.animate(gripperTraj(j,:));
            drawnow;
            else
                disp('Estop Triggered')
                input('Once work environent is safe press ENTER to resume operation')
                self.EmergencyFlag = false;
            end
        end
    end


    function rob1_RMRC_Jogging(self,linear,angular)
        % Have to input a "weighting" for movement linear and angular.
        % E.g. if linear [1,1,1] is entered, then the movement will make
        % increments of 1 in all three axes per function call.

        % 1.1) Set parameters for the simulation
        self.rob1.model.tool = transl(0,0,0);
        q = self.rob1.model.getpos;
        
        self.rmrc_step=self.rmrc_step+1; % increment step count
        
        % get values from the gui
        linear_motion = linear;
        angular_motion = angular;
           
        % -------------------------------------------------------------
        % YOUR CODE GOES HERE
        % 1 - turn joystick input into an end-effector force measurement  
        fx = linear_motion(1);
        fy = linear_motion(2);
        fz = linear_motion(3);
        
        tx = angular_motion(1);
        ty = angular_motion(2);
        tz = angular_motion(3);
        
        f = [fx;fy;fz;tx;ty;tz]; % combined force-torque vector (wrench)
        
        % 2 - use simple admittance scheme to convert force measurement into
        % velocity command
        Ka = diag([0.5,0.5,0.5,ones(1,3)]); % admittance gain matrix  
        dx = Ka*f; % convert wrench into end-effector velocity command
        
        % 2 - use DLS J inverse to calculate joint velocity
        J = self.rob1.model.jacobe(q);
        
        lambda = 0.1;
        Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
        dq = Jinv_dls*dx;
        
        % 3 - apply joint velocity to step robot joint angles
        q = q + dq';
        % -------------------------------------------------------------
        
        % Update plot
        self.rob1.model.animate(q); 
        drawnow();

    end


    function rob2_RMRC_Jogging(self,linear,angular)
        % Have to input a "weighting" for movement linear and angular.
        % E.g. if linear [1,1,1] is entered, then the movement will make
        % increments of 1 in all three axes per function call.

        % 1.1) Set parameters for the simulation
        self.rob2.model.tool = transl(0,0,0);
        q = self.rob2.model.getpos;
        
        self.rmrc_step=self.rmrc_step+1; % increment step count
        
        % get values from the gui
        linear_motion = linear;
        angular_motion = angular;
           
        % -------------------------------------------------------------
        % YOUR CODE GOES HERE
        % 1 - turn joystick input into an end-effector force measurement  
        fx = linear_motion(1);
        fy = linear_motion(2);
        fz = linear_motion(3);
        
        tx = angular_motion(1);
        ty = angular_motion(2);
        tz = angular_motion(3);
        
        f = [fx;fy;fz;tx;ty;tz]; % combined force-torque vector (wrench)
        
        % 2 - use simple admittance scheme to convert force measurement into
        % velocity command
        Ka = diag([0.5,0.5,0.5,ones(1,3)]); % admittance gain matrix  
        dx = Ka*f; % convert wrench into end-effector velocity command
        
        % 2 - use DLS J inverse to calculate joint velocity
        J = self.rob2.model.jacobe(q);
        
        lambda = 0.1;
        Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
        dq = Jinv_dls*dx;
        
        % 3 - apply joint velocity to step robot joint angles
        q = q + dq';
        % -------------------------------------------------------------
        
        % Update plot
        self.rob2.model.animate(q); 
        drawnow();

    end

    %% Environment Setup
        function setup_environment(self)
            hold on
            self.Truck = PlaceObject('NewTruckPlatform.ply',[0 0 0]);
            self.Apples(1,1) = PlaceObject('NewApple.ply',[0 1 1]);
            self.Apples(2,1) = PlaceObject('NewApple.ply',[0.3 1.3 0.7]);
            self.Apples(3,1) = PlaceObject('NewApple.ply',[0.5 1.2 1.1]);
            self.Apples(4,1) = PlaceObject('NewApple.ply',[0.7 1.2 0.9]);
            self.Apples(5,1) = PlaceObject('NewApple.ply',[0.4 1.1 1.5]);
            self.Apples(6,1) = PlaceObject('NewApple.ply',[0.75 1.2 1.75]);
            self.Apples(7,1) = PlaceObject('NewApple.ply',[1.4 1.1 1.25]);
            Tree1 = PlaceObject('NewTree.ply',[-0.5 1.4 0]);
            Tree2 = PlaceObject('NewTree.ply',[0.5 1.4 0]);
            Tree3 = PlaceObject('NewTree.ply',[1.5 1.4 0]);
            FireExtinguisher = PlaceObject('fireExtinguisher.ply',[1.5 0.22 0.4]);
            emergencyStopButton = PlaceObject('emergencyStopButton.ply', [1.6 -0.3 0.4]);

            self.rob1.model.base = transl([0.65 0.4 0.75]);
            self.rob2.model.base = transl([0.62 0.15 0.75]) * trotz(pi/2);
            self.rob1.model.animate(deg2rad([-90 0 0 0 0 0]));
            self.rob2.model.animate(deg2rad([0 0 0 0 0 0]));
            axis([-2 2 -2 2 0 2.2]);

            hold off

        end
    end

end