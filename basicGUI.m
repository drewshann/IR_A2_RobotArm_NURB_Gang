% Create a function to generate the GUI
function basicGUI()

    % Create a figure
    fig = figure('Position', [100, 100, 600, 400], 'Name', 'Basic GUI Example');
    subplot(2,1,1);
    plot([1,2,3,4],[1,2,3,4]);

    % Create a slider
    slider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 10, ...
        'Value', 5, 'Position', [0, 195, 150, 20], 'Callback', @sliderCallback);

    % Create a text label to display slider value
    textLabel = uicontrol('Style', 'text', 'Position', [160, 190, 50, 25], 'String', 'Value: 5');

    % Create a button to update the figure
    updateButton = uicontrol('Style', 'pushbutton', 'String', "Update", ...
        'Position', [0, 225, 50, 30], 'Callback', @updateButtonCallback);

    % Create a callback function for the slider
    function sliderCallback(source, ~)
        sliderValue = get(source, 'Value');
        set(textLabel, 'String', sprintf('Value: %.2f', sliderValue));
    end

    % Create a callback function for the button
    function updateButtonCallback(~, ~)
        sliderValue = get(slider, 'Value');
        % Use sliderValue to update your figure
        % For example, plot a function using sliderValue as a parameter
        x = linspace(0, 10, 100);
        y = sin(x * sliderValue);
        subplot(2,1,2);
        plot(x, y);
        title(['y = sin(x * ' num2str(sliderValue) ')']);
    end

end