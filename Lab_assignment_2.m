classdef Lab_assignment_2 < handle

    properties
        rob1;
        rob2;

    end

    properties(Constant)
        
    end

    methods
%% Constructor
        function self = Lab_assignment_2(robot1, robot2)
            % Create the environment within the constructor
            if nargin > 0
                self.rob1 = robot1;
                self.rob2 = robot2;     

            else

            end

            self.setup_environment();
        end


%% Change the Transform/pose relative to robot base into q values
        function q1 = transformation2Q_rob1(self, transform)
            % Have a try catch statement to tell whether the ikine function
            % converges or not (ie end effector position is achievable)

            % Run fkine after ikcon to test whether we got the correct end
            % effector position
            q1 = self.rob1.model.ikcon(transform);
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
            r2Traj = jtraj(start_q_rob2,end_q_rob2,trsteps);

            for i = 1:trsteps
                % r2.model.fkine(UR3.model.getpos)
                self.rob1.model.animate(r1Traj(i,:));
                self.rob2.model.animate(r2Traj(i,:));
                % self.update_gripper_pos();

                drawnow()       
            end
            
        end


    end

    methods(Static)

        function setup_environment()
            % Create the environment in here
        end
    end

end