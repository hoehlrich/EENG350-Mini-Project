%% run_position.m
% Closed loop position step response: simulation vs experiment, and a
% simulation showing why the controller needs integral action.
%
% Model: position_control.slx. The desired position steps from 0 to
% Step_Position (pi rad, goal 0 -> goal 1) at t = 1 s. A PI controller on
% the position error gives the desired speed, limited to +/- MAX_SPEED
% with clamping anti-windup. The velocity loop from assignment 2
% (Kp_vel, battery limit, motor model) turns that into wheel motion.
%
% Experiment: position_control.ino with STEP_TEST defined steps both
% wheels the same way. To record it, set both wheels to 0, upload, close
% the serial monitor, and run
%
%     readArduinoData("position_data.mat", 5)
%
% Columns: time, left voltage, left position, right voltage, right position.
% Without position_data.mat this script only plots the simulation.
%
% Tuning: start with Ki_pos = 0 and raise Kp_pos until the step is fast
% without overshoot, then add Ki_pos until disturbances are removed quickly
% (last section). Put the final gains in motor_params.m and in
% position_control.ino.
%
% Publish this script (publish('run_position.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
motor_params

dataFile = fullfile(here, 'position_data.mat');
haveData = isfile(dataFile);
if haveData
    load(dataFile, 'data')
else
    disp('No position_data.mat yet - plotting the simulation only.')
end

wheelNames = ["Left", "Right"];
Ks = [K1 K2];
sigmas = [sigma1 sigma2];

%% Step response for each wheel
figure
for w = 1:2
    in = Simulink.SimulationInput('position_control');
    in = in.setVariable('K', Ks(w));
    in = in.setVariable('sigma', sigmas(w));
    out = sim(in);

    subplot(2, 2, w)
    plot(out.DesiredPosition.Time, out.DesiredPosition.Data, 'k:', 'linewidth', 1.5)
    hold on
    plot(out.Position.Time, out.Position.Data, 'linewidth', 2)
    if haveData
        plot(data(:,1), data(:,2*w + 1), '--', 'linewidth', 2)
        legend('Desired', 'Simulation', 'Experiment', 'Location', 'southeast')
    else
        legend('Desired', 'Simulation', 'Location', 'southeast')
    end
    xlabel('Time (s)')
    ylabel('Position (rad)')
    title(wheelNames(w) + " wheel")

    subplot(2, 2, 2 + w)
    plot(out.Voltage.Time, out.Voltage.Data, 'linewidth', 2)
    hold on
    if haveData
        plot(data(:,1), data(:,2*w), '--', 'linewidth', 2)
        legend('Simulation', 'Experiment')
    end
    xlabel('Time (s)')
    ylabel('Voltage (V)')

    % Step response numbers from the simulation
    p = out.Position.Data;
    t = out.Position.Time;
    overshoot = 100 * (max(p) - Step_Position) / Step_Position;
    lastOut = find(abs(p - Step_Position) > 0.02*Step_Position, 1, 'last');
    fprintf('%s wheel (simulated): %.1f%% overshoot, 2%% settling time %.2f s\n', ...
            wheelNames(w), overshoot, t(lastOut) - Step_Time)
end

%% Disturbance rejection: P only vs PI
% A constant voltage disturbance, like a hand pushing on the wheel, is
% added at Disturbance_Time. With Ki_pos = 0 the wheel settles away from
% the goal. With integral action it is pulled back to exactly pi.
figure
gains = [0, Ki_pos];
labels = ["P only (Ki\_pos = 0)", sprintf("PI (Ki\\_pos = %g)", Ki_pos)];
plot([0 8], Step_Position*[1 1], 'k:', 'linewidth', 1.5)
hold on
for g = 1:2
    in = Simulink.SimulationInput('position_control');
    in = in.setModelParameter('StopTime', '8');
    in = in.setVariable('Ki_pos', gains(g));
    in = in.setVariable('Disturbance_V', -3);
    out = sim(in);
    plot(out.Position.Time, out.Position.Data, 'linewidth', 2)
end
xline(Disturbance_Time, '--', '-3 V disturbance')
legend(["Desired", labels], 'Location', 'southeast')
xlabel('Time (s)')
ylabel('Position (rad)')
title('Disturbance rejection (left wheel model)')
