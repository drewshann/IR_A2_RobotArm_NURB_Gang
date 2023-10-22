classdef GraphicalUserInterface
    
    properties
        fig1;
        fig2;
        robot;
        panel1;
        panel2;
        panel3;
        P1panel4;
        P2panel5;
        startButton;
        pauseButton;
        estopButton;
        resumeButton;
        resetButton;
        
        P1slider1;
        P1slider2;
        P1slider3;
        P1slider4;
        P1slider5;
        P1slider6;
        P1text1;
        P1text2;
        P1text3;
        P1text4;
        P1text5;
        P1text6;
        P1edit1;
        P1edit2;
        P1edit3;
        P1edit4;
        P1edit5;
        P1edit6;
        P1textX;
        P1textY;
        P1textZ;
        P1editX;
        P1editY;
        P1editZ;

        P2slider1;
        P2slider2;
        P2slider3;
        P2slider4;
        P2slider5;
        P2slider6;
        P2text1;
        P2text2;
        P2text3;
        P2text4;
        P2text5;
        P2text6;
        P2edit1;
        P2edit2;
        P2edit3;
        P2edit4;
        P2edit5;
        P2edit6;
        P2textX;
        P2textY;
        P2textZ;
        P2editX;
        P2editY;
        P2editZ;
    end
    
    methods
        function obj = GraphicalUserInterface()
            % fig1 = figure('Position', [100, 100, 600, 400], 'Name', 'Basic GUI Example');
            % % robot1 = UR3;
            % robot = UR3;

            obj.fig2 = figure('Position', [100, 100, 600, 400], 'Name', 'Fruit Picking GUI');
            obj.GUIsetup;
        end

        function GUIsetup(obj)
            obj.panel1 = uipanel(obj.fig2,'Position',[0.25 0.5125 0.725 0.4625],'Title','UR3','BorderType','etchedin','TitlePosition','centertop');
            % obj.panel1 = uipanel(obj.fig2,'Position',[0.25 0.025 0.35 0.95],'Title','UR3','BorderType','etchedin','TitlePosition','centertop');
            obj.panel2 = uipanel(obj.fig2,'Position',[0.25 0.025 0.725 0.4625],'Title','Robot','BorderType','etchedin','TitlePosition','centertop');
            % obj.panel2 = uipanel(obj.fig2,'Position',[0.625 0.025 0.35 0.95],'Title','Robot','BorderType','etchedin','TitlePosition','centertop');
            obj.panel3 = uipanel(obj.fig2,'Position',[0.025 0.025 0.2 0.95],'Title','Update Buttons','BorderType','etchedin','TitlePosition','centertop');
            
            obj.startButton = uicontrol('Style', 'pushbutton', 'String', "START", ...
                'Units', 'normalized', 'Position', [0.05, 0.775, 0.15, 0.1]);
            obj.pauseButton = uicontrol('Style', 'pushbutton', 'String', "PAUSE", ...
                'Units', 'normalized', 'Position', [0.05, 0.625, 0.15, 0.1]);
            obj.estopButton = uicontrol('Style', 'pushbutton', 'String', "E-Stop", ...
                'Units', 'normalized', 'Position', [0.05, 0.475, 0.15, 0.1]);
            obj.resumeButton = uicontrol('Style', 'pushbutton', 'String', "RESUME", ...
                'Units', 'normalized', 'Position', [0.05, 0.325, 0.15, 0.1]);
            obj.resetButton = uicontrol('Style', 'pushbutton', 'String', "RESET", ...
                'Units', 'normalized', 'Position', [0.05, 0.175, 0.15, 0.1]);

            obj.P1slider1 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 10, 'Units', 'normalized', 'Position', [0.32, 0.865, 0.28, 0.05], 'Callback', @obj.P1slidersCallback);
            obj.P1slider2 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.8, 0.28, 0.05]);
            obj.P1slider3 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.735, 0.28, 0.05]);
            obj.P1slider4 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.67, 0.28, 0.05]);
            obj.P1slider5 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.605, 0.28, 0.05]);
            obj.P1slider6 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.54, 0.28, 0.05]);
            obj.P1text1 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.87, 0.03, 0.04], 'String', 'q1:');
            obj.P1text2 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.805, 0.03, 0.04], 'String', 'q2:');
            obj.P1text3 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.74, 0.03, 0.04], 'String', 'q3:');
            obj.P1text4 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.675, 0.03, 0.04], 'String', 'q4:');
            obj.P1text5 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.61, 0.03, 0.04], 'String', 'q5:');
            obj.P1text6 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.545, 0.03, 0.04], 'String', 'q6:');
            obj.P1edit1 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.87, 0.075, 0.05], 'String', 0');
            obj.P1edit2 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.805, 0.075, 0.05], 'String', 0);
            obj.P1edit3 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.74, 0.075, 0.05], 'String', 0);
            obj.P1edit4 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.675, 0.075, 0.05], 'String', 0);
            obj.P1edit5 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.61, 0.075, 0.05], 'String', 0);
            obj.P1edit6 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.545, 0.075, 0.05], 'String', 0);

            obj.P2slider1 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.3775, 0.28, 0.05]);
            obj.P2slider2 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.3125, 0.28, 0.05]);
            obj.P2slider3 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.2475, 0.28, 0.05]);
            obj.P2slider4 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.1825, 0.28, 0.05]);
            obj.P2slider5 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.1175, 0.28, 0.05]);
            obj.P2slider6 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.0525, 0.28, 0.05]);
            obj.P2text1 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.3825, 0.03, 0.04], 'String', 'q1:');
            obj.P2text2 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.3175, 0.03, 0.04], 'String', 'q2:');
            obj.P2text3 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.2525, 0.03, 0.04], 'String', 'q3:');
            obj.P2text4 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.1875, 0.03, 0.04], 'String', 'q4:');
            obj.P2text5 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.1225, 0.03, 0.04], 'String', 'q5:');
            obj.P2text6 = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.28, 0.0575, 0.03, 0.04], 'String', 'q6:');
            obj.P2edit1 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.3825, 0.075, 0.05], 'String', 0);
            obj.P2edit2 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.3175, 0.075, 0.05], 'String', 0);
            obj.P2edit3 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.2525, 0.075, 0.05], 'String', 0);
            obj.P2edit4 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.1875, 0.075, 0.05], 'String', 0);
            obj.P2edit5 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.1225, 0.075, 0.05], 'String', 0);
            obj.P2edit6 = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.62, 0.0575, 0.075, 0.05], 'String', 0);

            obj.P1panel4 = uipanel(obj.fig2,'Position',[0.72 0.5375 0.23 0.4],'Title','X-Y-Z','BorderType','beveledin','TitlePosition','centertop');
            obj.P2panel5 = uipanel(obj.fig2,'Position',[0.72 0.05 0.23 0.4],'Title','X-Y-Z','BorderType','beveledin','TitlePosition','centertop');

            obj.P1textX = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.805, 0.03, 0.04], 'String', 'x:');
            obj.P1textY = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.705, 0.03, 0.04], 'String', 'y:');
            obj.P1textZ = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.605, 0.03, 0.04], 'String', 'z:');
            obj.P1editX = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.8, 0.075, 0.05], 'String', 0);
            obj.P1editY = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.7, 0.075, 0.05], 'String', 0);
            obj.P1editZ = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.6, 0.075, 0.05], 'String', 0);

            obj.P2textX = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.3175, 0.03, 0.04], 'String', 'x:');
            obj.P2textY = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.2175, 0.03, 0.04], 'String', 'y:');
            obj.P2textZ = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.77, 0.1175, 0.03, 0.04], 'String', 'z:');
            obj.P2editX = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.3125, 0.075, 0.05], 'String', 0);
            obj.P2editY = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.2125, 0.075, 0.05], 'String', 0);
            obj.P2editZ = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.8, 0.1125, 0.075, 0.05], 'String', 0);
        end

        function P1slidersCallback(obj,source,event)
            disp('check');
            val = get(obj.P1slider1, 'Value')
            val1 = get(source, 'Value')
            val2 = set(obj.P1edit1, 'Value', 100)
            set(obj.P1edit1, 'String', 'hello');
            val3 = get(source, 'Value')
            val4 = get(obj.P1edit1, 'Value')
        end
    end
end

