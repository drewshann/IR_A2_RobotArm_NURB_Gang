
           % Define the robot dimensions and joint limits
L1 = 1; L2 = 1; L3 = 1; L4 = 1; L5 = 1; L6 = 1;
qlim = [-pi, pi; -pi/2, pi/2; -pi, pi; -pi, pi; -pi/2, pi/2; -pi, pi];

% Define the robot using D-H parameters
robot = SerialLink([
    Link('d', 0, 'a', L1, 'alpha', pi/2, 'qlim', qlim(1,:)),
    Link('d', 0, 'a', L2, 'alpha', 0,    'qlim', qlim(2,:)),
    Link('d', 0, 'a', L3, 'alpha', 0,    'qlim', qlim(3,:)),
    Link('d', 0, 'a', L4, 'alpha', pi/2, 'qlim', qlim(4,:)),
    Link('d', 0, 'a', L5, 'alpha', -pi/2,'qlim', qlim(5,:)),
    Link('d', L6,'a', 0,  'alpha', 0,    'qlim', qlim(6,:))
], 'name', 'Basic 6-DOF Robot');

% Define the joint angles (all zeros in this case)
q = zeros(1,6);

% Define the plot options
plot_options = {'workspace', [-3 3 -3 3 0 3], 'scale', 0.5};

% Plot the robot
robot.plot(q, plot_options{:});


           