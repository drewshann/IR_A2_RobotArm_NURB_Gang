classdef KUKA < RobotBaseClass
    %% KR KUKA Aguilus

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
            %link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-166 166]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-188 45]), 'offset',0);
            link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-116 153]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-185 185]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-110,110]), 'offset',0);

             
            %link(1).qlim = [-0.8, -0.01];

            self.model = SerialLink(link,'name',"KUKA");
            q = zeros(1,5);
            workspace = [-1 1 -1 1 0 2];
            scale = 1;
            self.model.plot(q,'workspace',workspace,'scale',scale)

        end   

 
    end
end
