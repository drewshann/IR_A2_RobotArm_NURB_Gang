classdef KUKAKR < RobotBaseClass
    %% KR KUKA Aguilus

    properties(Access = public)   
        plyFileNameStem = 'KUKAKR';
    end
    
    methods
%% Constructor
function self = KUKAKR(baseTr)
           self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            % self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            %self.model.base = transl(0,0,0.5) * baseTr * trotx(pi/2) *   %troty(pi/2); testing
            
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1)=Link('alpha',-pi/2,'a',0, 'd',0.4, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
            link(2)=Link('alpha',0,'a',0.56, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
            link(3)=Link('alpha',pi/2,'a',-0.035, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
            link(4)=Link('alpha',-pi/2,'a',0, 'd',0.515, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
            link(5)=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
            link(6)=Link('alpha',0,'a',0, 'd',0.087, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);
            
            %link(1).qlim = [-0.8, -0.01];

            self.model = SerialLink(link,'name',self.name);
            % q = zeros(1,6);
            % workspace = [-1 1 -1 1 0 2];
            % scale = 1;
            % self.model.plot(q,'workspace',workspace,'scale',scale)

        end  

 
    end
end
