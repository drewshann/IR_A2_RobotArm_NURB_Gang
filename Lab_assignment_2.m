classdef Lab_assignment_2 < handle

    properties
        rob1;
        rob2;
        current_item;
        items_pos;
        cube;
        robot;
        Truck;
    end

    properties(Constant)
        trsteps = 50;
        initialGuessQRob1 = [-0.7849   -1.7363    0.0001    0.1327   -1.6231    0.7849];

        apple_pos1 = [deg2rad(43.2), deg2rad(-43.2), deg2rad(-14.4), deg2rad(28.8), deg2rad(43.2), deg2rad(28.8)];  % From teach
        test_single_pos = transl(0.4,0,0.2) * trotx(-pi);

        test_multiple_pos = {transl(0.4,0,0.3)*trotx(-pi), ...
                            transl(-0.4,0,0.3)*trotx(-pi), ...
                            transl(0.4,0.1,0.3)*trotx(-pi), ...
                            transl(-0.4,0.1,0.3)*trotx(-pi), ...
                            transl(-0.2,0.2,0.5)*trotx(-pi), ...
                            transl(0,0,0.6)*trotx(-pi)};

        ur3_link_sizes = {[0.05,0.05,0.1519], ...
                        [0.24365,0.05,0.05], ...
                        [0.21325,0.05,0.05], ...
                        [0.05,0.05,0.11235], ...
                        [0.05,0.05,0.08535], ...
                        [0.05,0.05,0.0819], ...
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
        function self = Lab_assignment_2(robot1, robot2)
            % Create the environment within the constructor
            if nargin > 0
                self.rob1 = robot1;
                self.rob2 = robot2;     

            else
                self.rob1 = UR5;
                self.items_pos = self.test_multiple_pos;

            end

            hold on
            % self.cube = PlaceObject("BasicObstacle.ply",[5.0,0,0]);
            self.setup_environment();
        end


        %% Change the Transform/pose relative to robot base into q values
        function q1 = transformation2Q_rob1(self, transform)
            % Have a try catch statement to tell whether the ikine function
            % converges or not (ie end effector position is achievable)

            % Run fkine after ikcon to test whether we got the correct end
            % effector position
            q1 = self.rob1.model.ikcon(transform, self.initialGuessQRob1);
        end

        function q2 = transformation2Q_rob2(self, transform)
            q2 = self.rob2.model.ikcon(transform);
        end

        %% Change the robot q values into an end effector transform
        function transform1 = q2Transform_rob1(self, q)
            transform1 = self.rob1.model.fkine(q);
        end

        function transform2 = q2Transform_rob2(self, q)
            transform2 = self.rob2.model.fkine(q);
        end


        %% Transform function, can be used to move the robots from one orientation to the next
        function transform_interpolation(self, start_q_rob1, end_q_rob1, start_q_rob2, end_q_rob2)

            % robot.model.getpos
            % UR3 = UR3_control.robot;
            
            r1Traj = jtraj(start_q_rob1,end_q_rob1,self.trsteps);
            % r2Traj = jtraj(start_q_rob2,end_q_rob2,trsteps);

            for i = 1:self.trsteps
                % r2.model.fkine(UR3.model.getpos)
                self.rob1.model.animate(r1Traj(i,:));
                % self.rob2.model.animate(r2Traj(i,:));
                % self.update_gripper_pos();
                self.checkCollisions();

                drawnow()
            end
            
        end

        %% Robotic control via Resolved Motion Rate Control
        function rmrc(self, start_q_rob1, end_q_rob1, start_q_rob2, end_q_rob2)
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
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
                x(2,i) = (1-s(i))*0 + s(i)*0.2; % Points in y
                x(3,i) = 0.25;% + 0.2*sin(i*delta); % Points in z
                theta(1,i) = 0;                 % Roll angle 
                theta(2,i) = 5*pi/9;            % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
            
            % Transform becomes imaginary here
            T1 = transl(x(1,1),x(1,2),x(1,3))*troty(theta(2,1));
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix(1,:) = self.rob1.model.ikcon(T1,q0);                                            % Solve joint angles to achieve first waypoint


            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
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
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.rob1.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.rob1.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                % angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            % 1.5) Plot the results
            tic
            
            % plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            for i = 1:steps
                self.rob1.model.animate(qMatrix(i,:));
                hold on
                end_eff = self.rob1.model.fkine(qMatrix(i,:));
                end_eff = end_eff.T;
                plot3(end_eff(1,4),end_eff(2,4),end_eff(3,4),'r.');
                drawnow();
            end
            
            disp(['Plot took ', num2str(toc), 'seconds'])
        end

        %% Function used to begin picking up apples autonomously
        function pickApples(self)
            num_items = size(self.test_multiple_pos);
% Idea for how to have both robots picking up apples at the same time:
% Have one big matrix for the positions of the apples and in that matrix
% also encode a boolean which tells whether it is currently being picked or
% not. So the cell array would look like this:
%   [transform for item position, true;
%   transform for item position, true;
%   transform for item position, false;
%   transform for item position, false]

            for i = 1:num_items(1,2)
                self.current_item = i;
               

                % Get current position and the q at that position
                qPos1 = self.rob1.model.getpos();

                % Gripper positioned z + 0.09 of brick.
                gripper_pos = self.items_pos{i};
                gripper_pos(3,4) = gripper_pos(3,4)+0.09;
                % Calc q at the end position
                qPos2 = self.transformation2Q_rob1(gripper_pos);
                disp(["The current brick is number: ",i]);
                disp("The current brick being picked up has the transform: ");
                disp(self.items_pos{i});
                
                % Open gripper and transform between current position and
                % starting brick position
                % self.open_gripper();
                self.transform_interpolation(qPos1,qPos2,qPos1,qPos1);


            end
        end


        %% Function used to check for collisions and return a true/false if the robot will collide with anything, including the other robot
        function collision = checkCollisions(self)
            
            EllipsoidCenterPoints = cell(1,7);
            transforms = cell(1,7);
            midpoints = cell(1,7);
            NumberOfLinks = size(self.rob1.model.links);
            disp(["Number of links on the robot: ", NumberOfLinks]);
            current_robq = self.rob1.model.getpos;

            % Initial ellipsoid centerpoint

            for i = 1:NumberOfLinks(1,2)+1
                [X,Y,Z] = ellipsoid(0,0,0,self.ur5_link_sizes{i}(1),self.ur5_link_sizes{i}(2),self.ur5_link_sizes{i}(3));
                EllipsoidPoints = [X(:),Y(:),Z(:)];

                transforms{i} = self.manual_fkine_rob1(current_robq,i);
                EllipsoidCenterPoints{i} = transforms{i}(1:3,4);
                disp(i);
                if i == 1
                    midpoints{i} = transforms{i}*trotx(-pi/2);
                    disp("The current link transform (pre midpoint adjustement) is");
                    disp(transforms{i});
                    midpoints{i}(1:3,4) = transforms{i}(1:3,4)/2;
                    disp("The current link transform (post midpoint adjustement) is");
                    disp(transforms{i});

                elseif i == 4
                    midpoints{i} = transforms{i}*trotx(-pi/2);
                    disp("The current link transform is");
                    disp(transforms{i});
                    disp("The previous link transform is");
                    disp(transforms{i-1});
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                    disp((transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2);

                elseif i == 5
                    midpoints{i} = transforms{i}*trotx(pi/2);
                    disp("The current link transform is");
                    disp(transforms{i});
                    disp("The previous link transform is");
                    disp(transforms{i-1});
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                    disp((transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2);
                else
                    midpoints{i} = transforms{i};
                    disp("The current link transform is");
                    disp(transforms{i});
                    disp("The previous link transform is");
                    disp(transforms{i-1});
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                    disp((transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2);
                end

                hold on
                EllipsoidPointsAndOnes = [midpoints{i} * [EllipsoidPoints,ones(size(EllipsoidPoints,1),1)]']';
                updatedEllipsoidPoints = EllipsoidPointsAndOnes(:,1:3);
                plot3(updatedEllipsoidPoints(:,1), EllipsoidPointsAndOnes(:,2), EllipsoidPointsAndOnes(:,3));
            end
            
            

            % verts = [get(self.Truck,'Vertices'), ones(size(get(self.Truck,'Vertices'),1), 1)];
            % verts(:,4) = [];
            % disp(verts);
            % 
            % % cubeAtOigin_h = plot3(verts(:,1),verts(:,2),verts(:,3),'r.');
            % 
            % for i = 1:NumberOfLinks(1,2)+1
            %     d = ((verts(:,1)-midpoints{i}(1,4))/self.ur3_link_sizes{i}(1)).^2 ...
            %       + ((verts(:,2)-midpoints{i}(2,4))/self.ur3_link_sizes{i}(2)).^2 ...
            %       + ((verts(:,3)-midpoints{i}(3,4))/self.ur3_link_sizes{i}(3)).^2;
            % 
            %     disp(d);
            %     for j = 1:size(d)
            %         disp(j)
            %         if d(j) < 1
            %             disp("intersection detected");
            %         end
            %     end
            % 
            % end
            

            % So the plan is (Check Lab6Solution question 2 and Lab6_collision_detection):
                % - Get the vertices of the ply file, if it's a complicated
                % shape then just use the straight vertices, if not then
                % create a mesh grid with reasonable number of points. Then
                % a point cloud of the object.

                % - Get the algebraic distance and check whether within 1

                % - If not then keep moving, if it is then stop. Might need
                % to actively avoid certain zones, gotta do stopping first.

                % Also need to create an ellipsoid around the robot arm

            collision = false;
        end

        %% Singularity Detection function and returns a cell array of the singularity locations
        function singularity = singularityLocations(self)

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
                Transform = Transform * linksArray{i};
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

    %% Environment Setup
        function setup_environment(self)
            hold on
            self.Truck = PlaceObject('NewTruckPlatform.ply',[0 0 0]);
            Apple1 = PlaceObject('NewApple.ply',[0 1.2 1]);
            Apple2 = PlaceObject('NewApple.ply',[1 1.2 1.2]);
            Apple3 = PlaceObject('NewApple.ply',[1.2 1.2 1.4]);
            Apple4 = PlaceObject('NewApple.ply',[0.5 1.2 1]);
            Apple5 = PlaceObject('NewApple.ply',[-0.5 1.2 1.5]);
            Apple6 = PlaceObject('NewApple.ply',[-1 1.2 1.2]);
            Tree1 = PlaceObject('NewTree.ply',[-1 1.5 0]);
            Tree2 = PlaceObject('NewTree.ply',[0 1.5 0]);
            Tree3 = PlaceObject('NewTree.ply',[1 1.5 0]);

            self.rob1.model.base = transl([0.5 0.4 0.75]);
            self.rob1.model.animate(deg2rad([0 -90 0 0 0 0]))
            axis([-2 2 -2 2 0 2.2]);
            hold off
        end
    end

end