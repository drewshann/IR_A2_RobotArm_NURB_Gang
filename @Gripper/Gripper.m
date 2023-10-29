classdef Gripper < RobotBaseClass
    %% Gripper for gripping apples

    properties(Access = public)              
        plyFileNameStem = 'Gripper';
    end
    
    methods
%% Define robot Function 
        function self = Gripper(baseTr)
            self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);
            end

            self.model.base = baseTr;

            % q = zeros(1,3);
            % q = [0,pi/3,pi/6];
            % workspace = [-0.2 0.2 -0.2 0.2 0 0.2];
            % scale = 1;
            % self.model.plot(q,'workspace', workspace, 'scale', scale);
            
            self.PlotAndColourRobot();
            self.model.animate([0,pi/3,pi/6]);

            drawnow
        end

    
    %% Create the robot model
        function CreateModel(self)
            Links(1) = Link('d',0,'a',0.03,'alpha',pi/2,'qlim',deg2rad([-0.0001 0.0001]), 'offset',0);
            Links(2) = Link('d',0,'a',0.05,'alpha',0,'qlim', deg2rad([0 180]), 'offset',0);
            Links(3) = Link('d',0,'a',0.04,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);

            self.model = SerialLink(Links,'name', self.name);

        end
    end
end