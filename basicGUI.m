% Create a function to generate the GUI
function basicGUI()

    % Create a figure
    % fig1 = figure('Position', [100, 100, 600, 400], 'Name', 'Basic GUI Example');
    robot = UR3;

    % fig2 = figure('Position', [100, 100, 600, 400], 'Name', 'Basic GUI Example');
    % p = uipanel(fig2,'Position',[0.05 0.05 0.9 0.9],'Title','Joint Angles','BorderType','beveledin','TitlePosition','centertop');
    % Create a slider
    slider1 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 220, 150, 20], 'Callback', @sliderCallback);
    slider2 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 200, 150, 20], 'Callback', @sliderCallback);
    slider3 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 180, 150, 20], 'Callback', @sliderCallback);
    slider4 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 160, 150, 20], 'Callback', @sliderCallback);
    slider5 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 140, 150, 20], 'Callback', @sliderCallback);
    slider6 = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Position', [0, 120, 150, 20], 'Callback', @sliderCallback);

    % Create a text label to display slider value
    textLabel1 = uicontrol('Style', 'edit', 'Position', [160, 220, 70, 25], 'String', 'q1: 0');
    textLabel2 = uicontrol('Style', 'edit', 'Position', [160, 200, 70, 25], 'String', 'q2: 0');
    textLabel3 = uicontrol('Style', 'edit', 'Position', [160, 180, 70, 25], 'String', 'q3: 0');
    textLabel4 = uicontrol('Style', 'edit', 'Position', [160, 160, 70, 25], 'String', 'q4: 0');
    textLabel5 = uicontrol('Style', 'edit', 'Position', [160, 140, 70, 25], 'String', 'q5: 0');
    textLabel6 = uicontrol('Style', 'edit', 'Position', [160, 120, 70, 25], 'String', 'q6: 0');

    % Create a button to update the figure
    updateButton = uicontrol('Style', 'pushbutton', 'String', "Update", ...
        'Position', [0, 255, 50, 30], 'Callback', @updateButtonCallback);

    % Create a callback function for the slider
    function sliderCallback(source, ~)
        sliderValue1 = get(slider1, 'Value');
        sliderValue2 = get(slider2, 'Value');
        sliderValue3 = get(slider3, 'Value');
        sliderValue4 = get(slider4, 'Value');
        sliderValue5 = get(slider5, 'Value');
        sliderValue6 = get(slider6, 'Value');
        set(textLabel1, 'String', sprintf('Value: %.2f', sliderValue1));
        set(textLabel2, 'String', sprintf('Value: %.2f', sliderValue2));
        set(textLabel3, 'String', sprintf('Value: %.2f', sliderValue3));
        set(textLabel4, 'String', sprintf('Value: %.2f', sliderValue4));
        set(textLabel5, 'String', sprintf('Value: %.2f', sliderValue5));
        set(textLabel6, 'String', sprintf('Value: %.2f', sliderValue6));
        robot.model.animate([deg2rad(sliderValue1),deg2rad(sliderValue2),deg2rad(sliderValue3),deg2rad(sliderValue4),deg2rad(sliderValue5),deg2rad(sliderValue6)]);
    end

    % Create a callback function for the button
    function updateButtonCallback(~, ~)
        sliderValue1 = get(slider1, 'Value');
        sliderValue2 = get(slider2, 'Value');
        sliderValue3 = get(slider3, 'Value');
        sliderValue4 = get(slider4, 'Value');
        sliderValue5 = get(slider5, 'Value');
        sliderValue6 = get(slider6, 'Value');
        % Use sliderValue to update your figure
        % For example, plot a function using sliderValue as a parameter
        % x = linspace(0, 10, 100);
        % y = sin(x * sliderValue);
        % title(['y = sin(x * ' num2str(sliderValue) ')']);
        robot.model.animate([deg2rad(sliderValue1),deg2rad(sliderValue2),deg2rad(sliderValue3),deg2rad(sliderValue4),deg2rad(sliderValue5),deg2rad(sliderValue6)]);
    end

end







% function joystickGUI
% 
%     % Create the main figure
%     mainFig = figure('Position', [100, 100, 400, 400], 'Name', 'Joystick GUI', 'NumberTitle', 'off');
% 
%     % Create buttons for directional control
%     upButton = uicontrol('Style', 'pushbutton', 'String', 'Up', ...
%                          'Units', 'normalized', 'Position', [0.45 0.8 0.1 0.1], ...
%                          'Callback', @moveUp);
% 
%     downButton = uicontrol('Style', 'pushbutton', 'String', 'Down', ...
%                            'Units', 'normalized', 'Position', [0.45 0.6 0.1 0.1], ...
%                            'Callback', @moveDown);
% 
%     leftButton = uicontrol('Style', 'pushbutton', 'String', 'Left', ...
%                            'Units', 'normalized', 'Position', [0.35 0.7 0.1 0.1], ...
%                            'Callback', @moveLeft);
% 
%     rightButton = uicontrol('Style', 'pushbutton', 'String', 'Right', ...
%                             'Units', 'normalized', 'Position', [0.55 0.7 0.1 0.1], ...
%                             'Callback', @moveRight);
% 
%     % Initialize a position indicator
%     positionText = uicontrol('Style', 'text', 'String', '(0, 0)', ...
%                             'Units', 'normalized', 'Position', [0.4 0.4 0.2 0.1]);
% 
%     % Initialize position variables
%     xPos = 0;
%     yPos = 0;
% 
%     % Callback functions for button presses
%     function moveUp(source, event)
%         yPos = yPos + 1;
%         updatePosition();
%     end
% 
%     function moveDown(source, event)
%         yPos = yPos - 1;
%         updatePosition();
%     end
% 
%     function moveLeft(source, event)
%         xPos = xPos - 1;
%         updatePosition();
%     end
% 
%     function moveRight(source, event)
%         xPos = xPos + 1;
%         updatePosition();
%     end
% 
%     % Update position text
%     function updatePosition()
%         set(positionText, 'String', sprintf('(%d, %d)', xPos, yPos));
%     end
% 
% end








% function draggableDotGUI
% 
%     % Create the main figure
%     mainFig = figure('Position', [100, 100, 400, 400], 'Name', 'Draggable Dot GUI', 'NumberTitle', 'off');
% 
%     % Create an axes to serve as the "box"
%     boxAxes = axes('Parent', mainFig, 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8], ...
%                    'XLim', [0 1], 'YLim', [0 1]);
% 
%     % Create a dot in the center of the box
%     dot = plot(boxAxes, 0.5, 0.5, 'ro', 'MarkerSize', 10);
% 
%     % Initialize variables for dragging
%     isDragging = false;
%     xOffset = 0;
%     yOffset = 0;
% 
%     % Set up callbacks for mouse events
%     set(mainFig, 'WindowButtonDownFcn', @startDrag, ...
%                  'WindowButtonMotionFcn', @drag, ...
%                  'WindowButtonUpFcn', @stopDrag);
% 
%     function startDrag(~, ~)
%         isDragging = true;
%         currentPoint = get(boxAxes, 'CurrentPoint');
%         xOffset = currentPoint(1, 1) - get(dot, 'XData');
%         yOffset = currentPoint(1, 2) - get(dot, 'YData');
%     end
% 
%     function drag(~, ~)
%         if isDragging
%             currentPoint = get(boxAxes, 'CurrentPoint');
%             newX = currentPoint(1, 1) - xOffset;
%             newY = currentPoint(1, 2) - yOffset;
%             set(dot, 'XData', newX, 'YData', newY);
%         end
%     end
% 
%     function stopDrag(~, ~)
%         isDragging = false;
%     end
% 
% end


