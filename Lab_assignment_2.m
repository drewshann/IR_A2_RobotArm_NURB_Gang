classdef Lab_assignment_2 < handle

    properties
        rob1;
        rob2;
        current_item;
        items_pos;
        cube;
    end

    properties(Constant)
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
                self.rob1 = UR3;
                self.items_pos = self.test_multiple_pos;

            end

            hold on
            self.cube = PlaceObject("BasicObstacle.ply",[5.0,0,0]);
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
        function transform(self, start_q_rob1, end_q_rob1, start_q_rob2, end_q_rob2)

            trsteps = 50;
            % robot.model.getpos
            % UR3 = UR3_control.robot;
            
            r1Traj = jtraj(start_q_rob1,end_q_rob1,trsteps);
            % r2Traj = jtraj(start_q_rob2,end_q_rob2,trsteps);

            for i = 1:trsteps
                % r2.model.fkine(UR3.model.getpos)
                self.rob1.model.animate(r1Traj(i,:));
                % self.rob2.model.animate(r2Traj(i,:));
                % self.update_gripper_pos();

                drawnow()       
            end
            
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
                self.transform(qPos1,qPos2,qPos1,qPos1);


            end
        end


        %% Function used to check for collisions and return a true/false if the robot will collide with anything, including the other robot
        function collision = checkCollisions(self)
            
            EllipsoidCenterPoints = cell(1,7);
            transforms = cell(1,7);
            midpoints = cell(1,7);
            NumberOfLinks = size(self.rob1.model.links);
            current_robq = self.rob1.model.getpos;

            % Initial ellipsoid centerpoint

            for i = 1:NumberOfLinks(1,2)+1
                [X,Y,Z] = ellipsoid(0,0,0,self.ur3_link_sizes{i}(1),self.ur3_link_sizes{i}(2),self.ur3_link_sizes{i}(3));
                EllipsoidPoints = [X(:),Y(:),Z(:)];

                transforms{i} = self.manual_fkine_rob1(current_robq,i);
                EllipsoidCenterPoints{i} = transforms{i}(1:3,4);
                disp(i);
                if i == 1
                    midpoints{i} = transforms{i};
                    disp("The current link transform (pre midpoint adjustement) is");
                    disp(transforms{i});
                    midpoints{i}(1:3,4) = transforms{i}(1:3,4)/2;
                    disp("The current link transform (post midpoint adjustement) is");
                    disp(transforms{i});

                else
                    midpoints{i} = transforms{i};
                    disp("The current link transform is");
                    disp(transforms{i});
                    disp("The previous link transform is");
                    disp(transforms{i-1});
                    midpoints{i}(1:3,4) = (transforms{i}(1:3,4) + transforms{i-1}(1:3,4))/2;
                    disp((transforms{i}(1:3,4) - transforms{i-1}(1:3,4))/2);
                end


                EllipsoidPointsAndOnes = [midpoints{i} * [EllipsoidPoints,ones(size(EllipsoidPoints,1),1)]']';
                updatedEllipsoidPoints = EllipsoidPointsAndOnes(:,1:3);
                plot3(updatedEllipsoidPoints(:,1), EllipsoidPointsAndOnes(:,2), EllipsoidPointsAndOnes(:,3));
            end
            
            

            % verts = [get(self.cube,'Vertices'), ones(size(get(self.cube,'Vertices'),1), 1)];
            % verts(:,4) = [];
            % disp(verts);
            % 
            % cubeAtOigin_h = plot3(verts(:,1),verts(:,2),verts(:,3),'r.');

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
                disp("The transform for the link number cannot be returned as it exceeds the UR3 robot's links");
                return
            end
            linksArray = {self.rob1.model.base, ...
                        trotz(q(1)) * transl(0,0,0.1519) * transl(0,0,0) * trotx(pi/2), ...
                        trotz(q(2)) * transl(0,0,0) * transl(-0.24365,0,0) * trotx(0), ...
                        trotz(q(3)) * transl(0,0,0) * transl(-0.21325,0,0) * trotx(0), ...
                        trotz(q(4)) * transl(0,0,0.11235) * transl(0,0,0) * trotx(pi/2), ...
                        trotz(q(5)) * transl(0,0,0.08535) * transl(0,0,0) * trotx(-pi/2), ...
                        trotz(q(6)) * transl(0,0,0.0819) * transl(0,0,0) * trotx(0), ...
                        eye(4)};
            % linksArray = {base_tr, joint1To2, joint2To3, joint3To4, joint4To5, joint5To6, joint6To7, tool_tr};
            

            Transform = linksArray{1}.T;
            for i = 2:return_link+1
                Transform = Transform * linksArray{i};
            end

            % End effector Transform = base_tr*joint0To1*joint1To2*joint2To3*joint3To4*joint4To5*joint5To6*joint6To7*tool_tr; 
        end


        function plot_robot_no_ply(self)
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            robot = SerialLink([L1,L2,L3,L4,L5,L6],'name','random_robot');

            robot.plot(zeros(1,6));
        end

    %% Environment Setup
        function setup_environment(self)
            hold on
            Truck = PlaceObject('NewTruckPlatform.ply',[0 0 0]);
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