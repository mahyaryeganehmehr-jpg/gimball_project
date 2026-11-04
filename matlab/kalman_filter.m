
clc;
clear;
close all;

%% Serial Configuration
port = "COM12";
baudRate = 115200;

s = serialport(port, baudRate);
configureTerminator(s, "CR/LF");
flush(s);

%% Kalman Filter Configuration

% Process noise covariance
Q = 0.01;

% Measurement noise covariance
R = 1;

% Initial state estimate
x_est = 0;

% Initial estimation error covariance
P = 1;

first_sample = true;

%% Plot Configuration
figure('Name', 'MPU6050 Gyroscope Z - Kalman Filter');

h_raw = animatedline('Color', 'r', 'LineWidth', 1);
h_filtered = animatedline('Color', 'b', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('MPU6050 Gyroscope Z - Raw vs Kalman');

legend('Raw Data', 'Kalman Filter');

t0 = tic;

disp("Receiving data... Press Ctrl+C to stop.");

%% Main Loop
while true

    line = readline(s);

    raw_value = str2double(strtrim(line));

    if ~isnan(raw_value)

        %% Convert raw data to deg/s
        gyro_z_dps = raw_value / 131;

        %% Kalman Filter

        if first_sample

            x_est = gyro_z_dps;
            P = R;

            first_sample = false;

        else

            % 1. Prediction
            x_pred = x_est;
            P_pred = P + Q;

            % 2. Kalman Gain
            K = P_pred / (P_pred + R);

            % 3. Correction
            x_est = x_pred + K * (gyro_z_dps - x_pred);

            % 4. Update covariance
            P = (1 - K) * P_pred;

        end

        %% Time
        t = toc(t0);

        %% Update Plot
        addpoints(h_raw, t, gyro_z_dps);
        addpoints(h_filtered, t, x_est);

        drawnow limitrate;

    end

end
