classdef GraphicalUserInterface
    
    properties
        assignment;
        % ur5;
        % kukaAgilus;

        fig1;
        fig2;

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
        P1upbuttonX;
        P1downbuttonX;
        P1upbuttonY;
        P1downbuttonY;
        P1upbuttonZ;
        P1downbuttonZ;
        P1opengrip;
        P1closegrip;
        P1startRMRC;
        P1stopRMRC;
        P1RMRC = 0;

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
        P2upbuttonX;
        P2downbuttonX;
        P2upbuttonY;
        P2downbuttonY;
        P2upbuttonZ;
        P2downbuttonZ;
        P2opengrip;
        P2closegrip;
        P2startRMRC;
        P2stopRMRC;
        P2RMRC;
    end

    properties(Constant)
        increment = 0.01;
    end
    
    methods
        function obj = GraphicalUserInterface()
            % obj.fig1 = figure('Position', [100, 100, 600, 400], 'Name', 'Fruit Picking Robot');
            % hold on
            % obj.ur5 = UR5;
            % obj.kukaAgilus = UR5;
            % hold off
            obj.fig1 = figure('Position', [100, 100, 600, 400], 'Name', 'Fruit Picking Robot');
            obj.assignment = Lab_assignment_2();

            obj.fig2 = figure('Position', [100, 100, 600, 400], 'Name', 'Fruit Picking GUI');
            obj = obj.GUIsetup();
            % figure(obj.fig1);
            obj.setupCallbacks();
        end

        function obj = GUIsetup(obj)
            obj.panel1 = uipanel(obj.fig2,'Position',[0.25 0.5125 0.725 0.4625],'Title','UR5','BorderType','etchedin','TitlePosition','centertop');
            % obj.panel1 = uipanel(obj.fig2,'Position',[0.25 0.025 0.35 0.95],'Title','UR5','BorderType','etchedin','TitlePosition','centertop');
            obj.panel2 = uipanel(obj.fig2,'Position',[0.25 0.025 0.725 0.4625],'Title','Kuka Agilus','BorderType','etchedin','TitlePosition','centertop');
            % obj.panel2 = uipanel(obj.fig2,'Position',[0.625 0.025 0.35 0.95],'Title','Kuka Agilus','BorderType','etchedin','TitlePosition','centertop');
            obj.panel3 = uipanel(obj.fig2,'Position',[0.025 0.025 0.2 0.95],'Title','Update Buttons','BorderType','etchedin','TitlePosition','centertop');
            
            obj.startButton = uicontrol('Style', 'pushbutton', 'String', "START", ...
                'Units', 'normalized', 'Position', [0.05, 0.775, 0.15, 0.1]);
            obj.pauseButton = uicontrol('Style', 'pushbutton', 'String', "PAUSE", ...
                'Units', 'normalized', 'Position', [0.05, 0.625, 0.15, 0.1]);
            obj.estopButton = uicontrol('Style', 'pushbutton', 'String', "E-STOP", ...
                'Units', 'normalized', 'Position', [0.05, 0.475, 0.15, 0.1]);
            obj.resumeButton = uicontrol('Style', 'pushbutton', 'String', "RESUME", ...
                'Units', 'normalized', 'Position', [0.05, 0.325, 0.15, 0.1]);
            obj.resetButton = uicontrol('Style', 'pushbutton', 'String', "RESET", ...
                'Units', 'normalized', 'Position', [0.05, 0.175, 0.15, 0.1]);

            obj.P1slider1 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.32, 0.865, 0.28, 0.05]);
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

            obj.P1panel4 = uipanel(obj.fig2,'Position',[0.72 0.59 0.23 0.3475],'Title','X-Y-Z','BorderType','beveledin','TitlePosition','centertop');
            obj.P2panel5 = uipanel(obj.fig2,'Position',[0.72 0.1025 0.23 0.3475],'Title','X-Y-Z','BorderType','beveledin','TitlePosition','centertop');

            obj.P1textX = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.83, 0.03, 0.04], 'String', 'x:');
            obj.P1textY = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.73, 0.03, 0.04], 'String', 'y:');
            obj.P1textZ = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.63, 0.03, 0.04], 'String', 'z:');
            obj.P1editX = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.825, 0.075, 0.05], 'String', 0);
            obj.P1editY = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.725, 0.075, 0.05], 'String', 0);
            obj.P1editZ = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.625, 0.075, 0.05], 'String', 0);
            obj.P1upbuttonX = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.85 0.035 0.025]);
            obj.P1downbuttonX = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.825 0.035 0.025]);
            obj.P1upbuttonY = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.75 0.035 0.025]);
            obj.P1downbuttonY = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.725 0.035 0.025]);
            obj.P1upbuttonZ = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.65 0.035 0.025]);
            obj.P1downbuttonZ = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.625 0.035 0.025]);
            obj.P1opengrip = uicontrol('Style','pushbutton','String',"Open Gripper",'Units', 'normalized', 'Position', [0.715 0.53 0.12 0.05]);
            obj.P1closegrip = uicontrol('Style','pushbutton','String',"Close Gripper",'Units', 'normalized', 'Position', [0.835 0.53 0.12 0.05]);
            obj.P1startRMRC = uicontrol('Style','pushbutton','String',"START RMRC",'FontSize',4,'Units', 'normalized', 'Position', [0.885 0.75 0.05 0.05]);
            obj.P1stopRMRC = uicontrol('Style','pushbutton','String',"STOP RMRC",'FontSize',4,'Units', 'normalized', 'Position', [0.885 0.65 0.05 0.05]);

            obj.P2textX = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.3425, 0.03, 0.04], 'String', 'x:');
            obj.P2textY = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.2425, 0.03, 0.04], 'String', 'y:');
            obj.P2textZ = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.73, 0.1425, 0.03, 0.04], 'String', 'z:');
            obj.P2editX = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.3375, 0.075, 0.05], 'String', 0);
            obj.P2editY = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.2375, 0.075, 0.05], 'String', 0);
            obj.P2editZ = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.76, 0.1375, 0.075, 0.05], 'String', 0);
            obj.P2upbuttonX = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.3625 0.035 0.025]);
            obj.P2downbuttonX = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.3375 0.035 0.025]);
            obj.P2upbuttonY = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.2625 0.035 0.025]);
            obj.P2downbuttonY = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.2375 0.035 0.025]);
            obj.P2upbuttonZ = uicontrol('Style','pushbutton','String',"▲",'FontSize',5,'Units','normalized','Position',[0.835 0.1625 0.035 0.025]);
            obj.P2downbuttonZ = uicontrol('Style','pushbutton','String',"▼",'FontSize',5,'Units','normalized','Position',[0.835 0.1375 0.035 0.025]);
            obj.P2opengrip = uicontrol('Style','pushbutton','String',"Open Gripper",'Units', 'normalized', 'Position', [0.715 0.0425 0.12 0.05]);
            obj.P2closegrip = uicontrol('Style','pushbutton','String',"Close Gripper",'Units', 'normalized', 'Position', [0.835 0.0425 0.12 0.05]);
            obj.P2startRMRC = uicontrol('Style','pushbutton','String',"START RMRC",'FontSize',4,'Units', 'normalized', 'Position', [0.885 0.28 0.05 0.05]);
            obj.P2stopRMRC = uicontrol('Style','pushbutton','String',"STOP RMRC",'FontSize',4,'Units', 'normalized', 'Position', [0.885 0.18 0.05 0.05]);
        end

        function setupCallbacks(obj)
            % Set up callbacks
            addlistener(obj.startButton, 'Value', 'PostSet', @obj.startButtonCallback);
            addlistener(obj.pauseButton, 'Value', 'PostSet', @obj.pauseButtonCallback);
            addlistener(obj.estopButton, 'Value', 'PostSet', @obj.estopButtonCallback);
            addlistener(obj.resumeButton, 'Value', 'PostSet', @obj.resumeButtonCallback);
            addlistener(obj.resetButton, 'Value', 'PostSet', @obj.resetButtonCallback);

            P1sliders = [obj.P1slider1,obj.P1slider2,obj.P1slider3,obj.P1slider4,obj.P1slider5,obj.P1slider6];
            P2sliders = [obj.P2slider1,obj.P2slider2,obj.P2slider3,obj.P2slider4,obj.P2slider5,obj.P2slider6];
            addlistener(P1sliders, 'Value', 'PostSet', @obj.P1slidersCallback);
            addlistener(P2sliders, 'Value', 'PostSet', @obj.P2slidersCallback);

            addlistener(obj.P1upbuttonX, 'Value', 'PostSet', @obj.P1upXCallback);
            addlistener(obj.P1downbuttonX, 'Value', 'PostSet', @obj.P1downXCallback);
            addlistener(obj.P1upbuttonY, 'Value', 'PostSet', @obj.P1upYCallback);
            addlistener(obj.P1downbuttonY, 'Value', 'PostSet', @obj.P1downYCallback);
            addlistener(obj.P1upbuttonZ, 'Value', 'PostSet', @obj.P1upZCallback);
            addlistener(obj.P1downbuttonZ, 'Value', 'PostSet', @obj.P1downZCallback);
            addlistener(obj.P1startRMRC, 'Value', 'PostSet', @obj.P1RMRCstart);
            addlistener(obj.P1stopRMRC, 'Value', 'PostSet', @obj.P1RMRCstop);
            addlistener(obj.P1opengrip, 'Value', 'PostSet', @obj.P1openGripper);
            addlistener(obj.P1closegrip, 'Value', 'PostSet', @obj.P1closeGripper);

            addlistener(obj.P2upbuttonX, 'Value', 'PostSet', @obj.P2upXCallback);
            addlistener(obj.P2downbuttonX, 'Value', 'PostSet', @obj.P2downXCallback);
            addlistener(obj.P2upbuttonY, 'Value', 'PostSet', @obj.P2upYCallback);
            addlistener(obj.P2downbuttonY, 'Value', 'PostSet', @obj.P2downYCallback);
            addlistener(obj.P2upbuttonZ, 'Value', 'PostSet', @obj.P2upZCallback);
            addlistener(obj.P2downbuttonZ, 'Value', 'PostSet', @obj.P2downZCallback);
            addlistener(obj.P2startRMRC, 'Value', 'PostSet', @obj.P2RMRCstart);
            addlistener(obj.P2stopRMRC, 'Value', 'PostSet', @obj.P2RMRCstop);
            addlistener(obj.P2opengrip, 'Value', 'PostSet', @obj.P2openGripper);
            addlistener(obj.P2closegrip, 'Value', 'PostSet', @obj.P2closeGripper);

            P1edits = [obj.P1edit1,obj.P1edit2,obj.P1edit3,obj.P1edit4,obj.P1edit5,obj.P1edit6];
            P2edits = [obj.P2edit1,obj.P2edit2,obj.P2edit3,obj.P2edit4,obj.P2edit5,obj.P2edit6];
            addlistener(P1edits, 'String', 'PostSet', @obj.P1editsCallback);
            addlistener(P2edits, 'String', 'PostSet', @obj.P2editsCallback);

            P1xyz = [obj.P1editX, obj.P1editY, obj.P1editZ];
            P2xyz = [obj.P2editX, obj.P2editY, obj.P2editZ];
            addlistener(P1xyz, 'String', 'PostSet', @obj.P1xyzCallback);
            addlistener(P2xyz, 'String', 'PostSet', @obj.P2xyzCallback);
        end

        function startButtonCallback(obj,~,~)
            % Startup fruit picking imlpementation
            set(obj.P1slider1, 'Value', -90);
            set(obj.P1slider2, 'Value', -90);
            set(obj.P1slider3, 'Value', 0);
            set(obj.P1slider4, 'Value', 0);
            set(obj.P1slider5, 'Value', 0);
            set(obj.P1slider6, 'Value', 0);

            obj.assignment.pickApples(obj.fig1);
        end

        function pauseButtonCallback(obj,~,~)
            % Pause running of fruit picking implementation
        end

        function estopButtonCallback(obj,~,~)
            % Emergency stop implementation
            obj.assignment.EmergencyFlag = True;
        end

        function resumeButtonCallback(obj,~,~)
            % Begin from where we stoped or paused implementation
            obj.assignment.EmergencyFlag = False;
        end

        function resetButtonCallback(obj,~,~)
            % Reset to beginning of fruit picking implementation
        end

        function P1slidersCallback(obj,~,~)
            % Implement functionality to change slider automatically while it is moving during startup mission
            % Connect slider values to ur5 robot in mission
            val1 = get(obj.P1slider1, 'Value');
            val2 = get(obj.P1slider2, 'Value');
            val3 = get(obj.P1slider3, 'Value');
            val4 = get(obj.P1slider4, 'Value');
            val5 = get(obj.P1slider5, 'Value');
            val6 = get(obj.P1slider6, 'Value');
            set(obj.P1edit1, 'String', num2str(round(val1,2)));
            set(obj.P1edit2, 'String', num2str(round(val2,2)));
            set(obj.P1edit3, 'String', num2str(round(val3,2)));
            set(obj.P1edit4, 'String', num2str(round(val4,2)));
            set(obj.P1edit5, 'String', num2str(round(val5,2)));
            set(obj.P1edit6, 'String', num2str(round(val6,2)));

            obj.assignment.rob1.model.animate(deg2rad([val1,val2,val3,val4,val5,val6]));
            obj.assignment.moveGripper1(1);
        end

        function P2slidersCallback(obj,~,~)
            % Implement functionality to change slider automatically while it is moving during startup mission
            % Connect slider values to 2nd robot in mission
            val1 = get(obj.P2slider1, 'Value');
            val2 = get(obj.P2slider2, 'Value');
            val3 = get(obj.P2slider3, 'Value');
            val4 = get(obj.P2slider4, 'Value');
            val5 = get(obj.P2slider5, 'Value');
            val6 = get(obj.P2slider6, 'Value');
            set(obj.P2edit1, 'String', num2str(round(val1,2)));
            set(obj.P2edit2, 'String', num2str(round(val2,2)));
            set(obj.P2edit3, 'String', num2str(round(val3,2)));
            set(obj.P2edit4, 'String', num2str(round(val4,2)));
            set(obj.P2edit5, 'String', num2str(round(val5,2)));
            set(obj.P2edit6, 'String', num2str(round(val6,2)));

            obj.assignment.rob2.model.animate(deg2rad([val1,val2,val3,val4,val5,val6]));
            obj.assignment.moveGripper2(1);
        end

        function P1editsCallback(obj,~,~)
            % Make sure it is between min,max
            set(obj.P1slider1, 'Value', str2double(get(obj.P1edit1, 'String')));
            set(obj.P1slider2, 'Value', str2double(get(obj.P1edit2, 'String')));
            set(obj.P1slider3, 'Value', str2double(get(obj.P1edit3, 'String')));
            set(obj.P1slider4, 'Value', str2double(get(obj.P1edit4, 'String')));
            set(obj.P1slider5, 'Value', str2double(get(obj.P1edit5, 'String')));
            set(obj.P1slider6, 'Value', str2double(get(obj.P1edit6, 'String')));
        end

        function P2editsCallback(obj,~,~)
            % Make sure it is between min,max
            set(obj.P2slider1, 'Value', str2double(get(obj.P2edit1, 'String')));
            set(obj.P2slider2, 'Value', str2double(get(obj.P2edit2, 'String')));
            set(obj.P2slider3, 'Value', str2double(get(obj.P2edit3, 'String')));
            set(obj.P2slider4, 'Value', str2double(get(obj.P2edit4, 'String')));
            set(obj.P2slider5, 'Value', str2double(get(obj.P2edit5, 'String')));
            set(obj.P2slider6, 'Value', str2double(get(obj.P2edit6, 'String')));
        end

        function P1upXCallback(obj,~,~)
            set(obj.P1editX, 'String', num2str(str2double(get(obj.P1editX, 'String')) + obj.increment));
        end
        function P1downXCallback(obj,~,~)
            set(obj.P1editX, 'String', num2str(str2double(get(obj.P1editX, 'String')) - obj.increment));
        end
        function P1upYCallback(obj,~,~)
            set(obj.P1editY, 'String', num2str(str2double(get(obj.P1editY, 'String')) + obj.increment));
        end
        function P1downYCallback(obj,~,~)
            set(obj.P1editY, 'String', num2str(str2double(get(obj.P1editY, 'String')) - obj.increment));
        end
        function P1upZCallback(obj,~,~)
            set(obj.P1editZ, 'String', num2str(str2double(get(obj.P1editZ, 'String')) + obj.increment));
        end
        function P1downZCallback(obj,~,~)
            set(obj.P1editZ, 'String', num2str(str2double(get(obj.P1editZ, 'String')) - obj.increment));
        end
        function obj = P1RMRCstart(obj,~,~)
            obj.P1RMRC = 1;
            while obj.P1RMRC == 1
                X = str2double(get(obj.P1editX, 'String'));
                Y = str2double(get(obj.P1editY, 'String'));
                Z = str2double(get(obj.P1editZ, 'String'));

                obj.assignment.rob1_RMRC_Jogging([X,Y,Z],[0 0 0]);
                obj.assignment.moveGripper1(1);
            end
        end
        function obj = P1RMRCstop(obj,~,~)
            obj.P1RMRC = 0;
            set(obj.P1editX, 'String', "0");
            set(obj.P1editY, 'String', "0");
            set(obj.P1editZ, 'String', "0");
        end
        function P1openGripper(obj,~,~)
            obj.assignment.opencloseGripper1(2);
        end
        function P1closeGripper(obj,~,~)
            obj.assignment.opencloseGripper1(1);
        end

        function P2upXCallback(obj,~,~)
            set(obj.P2editX, 'String', num2str(str2double(get(obj.P2editX, 'String')) + obj.increment));
        end
        function P2downXCallback(obj,~,~)
            set(obj.P2editX, 'String', num2str(str2double(get(obj.P2editX, 'String')) - obj.increment));
        end
        function P2upYCallback(obj,~,~)
            set(obj.P2editY, 'String', num2str(str2double(get(obj.P2editY, 'String')) + obj.increment));
        end
        function P2downYCallback(obj,~,~)
            set(obj.P2editY, 'String', num2str(str2double(get(obj.P2editY, 'String')) - obj.increment));
        end
        function P2upZCallback(obj,~,~)
            set(obj.P2editZ, 'String', num2str(str2double(get(obj.P2editZ, 'String')) + obj.increment));
        end
        function P2downZCallback(obj,~,~)
            set(obj.P2editZ, 'String', num2str(str2double(get(obj.P2editZ, 'String')) - obj.increment));
        end
        function obj = P2RMRCstart(obj,~,~)
            obj.P2RMRC = 1;
            while obj.P2RMRC == 1
                X = str2double(get(obj.P2editX, 'String'));
                Y = str2double(get(obj.P2editY, 'String'));
                Z = str2double(get(obj.P2editZ, 'String'));

                obj.assignment.rob2_RMRC_Jogging([X,Y,Z],[0 0 0]);
                obj.assignment.moveGripper2(1);
            end
        end
        function obj = P2RMRCstop(obj,~,~)
            obj.P2RMRC = 0;
            set(obj.P2editX, 'String', "0");
            set(obj.P2editY, 'String', "0");
            set(obj.P2editZ, 'String', "0");
        end
        function P2openGripper(obj,~,~)
            obj.assignment.opencloseGripper2(2);
        end
        function P2closeGripper(obj,~,~)
            obj.assignment.opencloseGripper2(1);
        end

        function P1xyzCallback(obj,~,~)
            % Set up usage of this with model and program (use ikcon/ikine)
            R1x = str2double(get(obj.P1editX, 'String'));
            R1y = str2double(get(obj.P1editY, 'String'));
            R1z = str2double(get(obj.P1editZ, 'String'));
        end

        function P2xyzCallback(obj,~,~)
            % Set up usage of this with model and program (use ikcon/ikine)
            R2x = str2double(get(obj.P2editX, 'String'));
            R2y = str2double(get(obj.P2editY, 'String'));
            R2z = str2double(get(obj.P2editZ, 'String'));
        end
    end
end












% classdef SliderGUI < handle
% 
%     properties
%         f;       % Figure handle
%         slider;  % Slider handle
%         text;    % Text box handle
%     end
% 
%     methods
%         function obj = SliderGUI()
%             % Constructor: Create GUI
%             obj.createGUI();
%             obj.setupCallbacks();
%         end
% 
%         function createGUI(obj)
%             % Create GUI components
%             obj.f = figure('Position', [100, 100, 600, 400], 'Name', 'Slider GUI Example');
%             obj.slider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 100, ...
%                 'Value', 50, 'Position', [50, 150, 300, 20]);
%             obj.text = uicontrol('Style', 'text', 'Position', [200, 250, 100, 30], ...
%                 'String', '50', 'FontSize', 14);
%         end
% 
%         function setupCallbacks(obj)
%             % Set up callbacks
%             addlistener(obj.slider, 'Value', 'PostSet', @obj.sliderCallback);
%         end
% 
%         function sliderCallback(obj, ~, ~)
%             % Slider callback function
%             value = get(obj.slider, 'Value');
%             set(obj.text, 'String', num2str(value));
%         end
%     end
% end