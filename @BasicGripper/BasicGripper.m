classdef BasicGripper < handle
    %% LinearUR3 UR3 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'BasicGripper';
        left_model;
        right_model;
    end
    
    methods
%% Define robot Function 
        function self = BasicGripper()
			self.CreateGripper();
        end

    
    %% Create the robot model
        function CreateGripper(self)
            left(1) = Link('d',0,'a',0.03,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            left(2) = Link('d',0,'a',0.05,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',0);
            left(3) = Link('d',0,'a',0.04,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);


            right(1) = Link('d',0,'a',-0.03,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            right(2) = Link('d',0,'a',-0.05,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',0);
            right(3) = Link('d',0,'a',-0.04,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);

            self.left_model = SerialLink(left,'name','finger1');
            self.right_model = SerialLink(right,'name','finger2');
            % self.left_model.teach();


            % q = zeros(1,3);
            q = [0,pi/3,pi/6];
            workspace = [-0.2 0.2 -0.2 0.2 0 0.2];
            scale = 1;

            self.left_model.plot(q,'workspace', workspace, 'scale', scale);
            hold on
            self.right_model.plot(-q,'workspace', workspace, 'scale', scale);
            % self.left_model.animate(self.left_model.getpos);
            % self.right_model.animate(self.right_model.getpos);

        end
    end
end