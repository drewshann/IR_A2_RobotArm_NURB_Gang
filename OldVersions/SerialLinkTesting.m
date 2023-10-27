
            link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-180 1]), 'offset',0);
            link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             

            robot = SerialLink(link,'name',"KUKA");
            q = zeros(1,6);
            workspace = [-1 1 -1 1 0 2];
            scale = 1;
            plot(q,'workspace',workspace,'scale',scale)

           