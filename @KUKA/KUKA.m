classdef KUKA < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'KUKA';
    end
    
    methods
%% Constructor
function self = KUKA(baseTr)
           self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            %self.model.base = transl(0,0,0.5) * baseTr * trotx(pi/2) *   %troty(pi/2); testing
            
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('a',0,'alpha',-pi/2,'theta',pi/2,'qlim',deg2rad([-170 170]), 'offset',0);
            link(2) = Link('a',-0.24365,'alpha',0,'theta',-pi/2,'qlim', deg2rad([-190 45]), 'offset',0);
            link(3) = Link('d',0,'a',-0.21325,'alpha',pi/2,'qlim', deg2rad([-120 156]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',-pi/2,'qlim',deg2rad([-185 185]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',pi/2,'qlim',deg2rad([-120,120]), 'offset',0);
            link(6) = Link('d',0.0819,'a',0,'alpha',pi,'qlim',deg2rad([-350,350]), 'offset', 0);
             

self.model = SerialLink(link, 'name', self.name);

        end   

 %% Implementing IK Maybe?????/
       % function q = solveIK(self, targetTransform)
       %      q = self.model.ikine(targetTransform);
       % end 

    end
end
