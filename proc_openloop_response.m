%% USV Open-Loop Surge Step Response
% System identification of surge (forward velocity) model
% Input:  RCOU_C1 — throttle command to ESC (%)
% Output: GPS_Spd — ground speed (m/s)

clear; clc; close all;

% ── Load ─────────────────────────────────────────────────────────────────
% Linux/Mac path
% fname = "/home/bsb/Downloads/00000008.BIN.mat";
% Windows path
fname = 'C:\Users\bsb\Downloads\00000010.BIN.mat';
fprintf('Loading %s...\n', fname);
d = load(fname);

%% ── Time bases (zero-referenced) ─────────────────────────────────────────
t0      = d.ATT_timestamp(1);
t_rcou  = d.RCOU_timestamp;% - t0;
t_gps   = d.GPS_timestamp;%  - t0;

% ── Plot Speed Response ─────────────────────────────────────────────────────────────────
figure(1);
clf()
%yyaxis left
ax1 = subplot(211);
plot(d.RCOU_timestamp, d.RCOU_C3, 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'Throttle Command');
ylabel('Throttle Command (PWM)');
grid on;
title('Open-Loop Surge Step Response — Throttle Command vs Ground Speed');

ax2 = subplot(212);
%yyaxis right
plot(d.GPS_timestamp, d.GPS_Spd, 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Ground Speed (m/s)');
ylabel('Ground Speed (m/s)');

xlabel('Time (s)');
grid on;

% Link the x-axes of both subplots
linkaxes([ax1, ax2], 'x'); 

% sgtitle('Surge System ID — Open Loop Step Response', ...
%         'FontSize', 14, 'FontWeight', 'bold');
% 
% 
% fprintf('GPS sample rate: %.1f Hz\n', ...
%         1/mean(diff(t_gps)));
% fprintf('RCOU sample rate: %.1f Hz\n', ...
%         1/mean(diff(t_rcou)));
% fprintf('Max ground speed: %.2f m/s\n', max(gps_spd));
% fprintf('Max throttle:     %.1f%%\n',   max(throttle_pct));


%% Yaw-Rate Rtep Response
% USV Open-Loop Yaw Step Response
% System identification of yaw rate model
% Input:  RCOU_C3 — rudder command to servo (%)
% Output: IMU_GyrZ — yaw rate (rad/s converted to deg/s)

% ── Time bases (zero-referenced) ─────────────────────────────────────────
t_rcou = d.RCOU_timestamp - t0;
t_imu  = d.IMU_timestamp  - t0;

% ── Signals ──────────────────────────────────────────────────────────────
rudder_pct = d.RCOU_C3; %d.RCOU_C3_pct_rudder;            % -100 to +100%
yaw_rate   = d.IMU_GyrZ * (180/pi);           % rad/s → deg/s

% ── Plot ─────────────────────────────────────────────────────────────────
figure(2)
clf()
ax1 = subplot(211);
plot(d.RCOU_timestamp, d.RCOU_C1, 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'Rudder Command (%)');
ylabel('Rudder Command (PWM)');
title('Open-Loop Yaw Step Response — Rudder Command vs Yaw Rate');
grid on;

ax2 = subplot(212);
plot(d.IMU_timestamp, d.IMU_GyrZ * (180/pi), 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Yaw Rate (deg/s)');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (s)');
legend('Location', 'best');
grid on;
linkaxes([ax1, ax2], 'x'); 

