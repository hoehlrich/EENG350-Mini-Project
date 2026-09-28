%% motor_params.m
% Parameters shared by every control script and Simulink model in this
% folder. Run it (the run_*.m scripts do this for you) before simulating
% openloop_sim.slx or position_control.slx by hand.
%
% Keep the controller values in sync with the constants at the top of
% control/position_control/position_control.ino.

%% Motor models: velocity(s)/voltage(s) = K*sigma/(s + sigma)
% Measure with step_response.ino, readArduinoData, and run_openloop.m,
% which prints fitted values to paste here.
K1     = 2.2;    % left wheel DC gain (rad/s per V)
sigma1 = 5;      % left wheel pole (1/s)
K2     = 2.2;    % right wheel DC gain (rad/s per V)
sigma2 = 5;      % right wheel pole (1/s)

% The models use K and sigma. The run scripts set these per wheel; these
% defaults let the models run straight from the Simulink editor.
K     = K1;
sigma = sigma1;

%% Controller (same names as position_control.ino)
Kp_vel    = 3.0;   % velocity loop gain (V per rad/s), from assignment 2
Kp_pos    = 10.0;   % position proportional gain ((rad/s) per rad), tuned in run_position.m
Ki_pos    = 10.0;   % position integral gain ((rad/s) per rad*s), tuned in run_position.m
MAX_SPEED = 6.0;   % limit on the desired speed from the position loop (rad/s)

Battery_Voltage = 7.8;   % V, limits the motor voltage
Ts = 0.01;               % Arduino loop time (s)

%% Experiment settings
% Open loop: voltage step from 0 to Step_Voltage at t = 1 s (step_response.ino)
Step_Voltage = 3.0;      % V
% Closed loop: position goal step from 0 to Step_Position at t = 1 s
% (position_control.ino with STEP_TEST defined, goal 1 = pi rad)
Step_Position = pi;      % rad
Step_Time = 1.0;         % s

% Voltage disturbance added at the motor input, to show integral action.
% Zero for the normal step response; run_position.m sets it for its
% disturbance section.
Disturbance_V    = 0;    % V
Disturbance_Time = 3.0;  % s
