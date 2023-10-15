classdef Lab_assignment_2 < handle

    properties
        rob1;
        rob2;
        current_item;
        items_pos;
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
            
            
        end

%% Singularity Detection function and returns a cell array of the singularity locations
        function singularity = singularityLocations(self)

        end


    end

    methods(Static)

        function setup_environment()
            % Create the environment in here
        end
    end

end