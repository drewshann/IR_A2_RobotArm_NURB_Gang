%% Robotics
% Updated syntax and tested for V10 compatibility (Jan 2023)

% Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
function RMRC_Jogging_test(linear,angular)
  
    % 1.1) Set parameters for the simulation
    ur5 = UR5;        % Load robot model
    ur5.model.tool = transl(0,0,0);
    q = zeros(1,6);
    
    n = 0 % Initial step count
    
    n=n+1; % increment step count
    
    % get values from the gui
    linear_motion = [1,1,1];
    angular_motion = [pi/2,0,0];
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector force measurement  
    fx = linear(1);
    fy = linear(2);
    fz = linear(3);
    
    tx = angular(1);
    ty = angular(2);
    tz = angular(3);
    
    f = [fx;fy;fz;tx;ty;tz]; % combined force-torque vector (wrench)
    
    % 2 - use simple admittance scheme to convert force measurement into
    % velocity command
    Ka = diag(ones(1,6)); % admittance gain matrix  
    dx = Ka*f; % convert wrench into end-effector velocity command
    
    % 2 - use DLS J inverse to calculate joint velocity
    J = robot.jacobe(q);
    
    lambda = 0.1;
    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
    dq = Jinv_dls*dx;
    
    % 3 - apply joint velocity to step robot joint angles
    q = q + dq'*dt;
    % -------------------------------------------------------------
    
    % Update plot
    robot.animate(q); 

end