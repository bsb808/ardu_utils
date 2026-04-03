%% USV Open-Loop Surge Step Response
% System identification of surge (forward velocity) model
% Input:  RCOU_C1 — throttle command to ESC (%)
% Output: GPS_Spd — ground speed (m/s)

clear; clc; close all;

%% ── Load ─────────────────────────────────────────────────────────────────
fname = "/home/bsb/Downloads/00000008.BIN.mat";
fprintf('Loading %s...\n', fname);
d = load(fname);

%% ── Time bases (zero-referenced) ─────────────────────────────────────────
t0      = d.ATT_timestamp(1);
t_rcou  = d.RCOU_timestamp - t0;
t_gps   = d.GPS_timestamp  - t0;

%% ── Signals ──────────────────────────────────────────────────────────────
throttle_pct = d.RCOU_C1_pct_throttle;   % 0 to 100%
gps_spd      = d.GPS_Spd;                % ground speed (m/s)

%% ── Plot ─────────────────────────────────────────────────────────────────
fig = figure('Name', 'USV Open-Loop Surge Step Response', ...
             'Position', [100 100 1100 500]);

yyaxis left
plot(t_rcou, throttle_pct, 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'Throttle Command (%)');
ylabel('Throttle Command (%)');
ylim([-5 110]);
yline(0, 'b:', 'LineWidth', 0.75);

yyaxis right
plot(t_gps, gps_spd, 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Ground Speed (m/s)');
ylabel('Ground Speed (m/s)');
ylim([0, max(gps_spd) * 1.2]);

xlabel('Time (s)');
title('Open-Loop Surge Step Response — Throttle Command vs Ground Speed');
legend('Location', 'best');
grid on;

sgtitle('Surge System ID — Open Loop Step Response', ...
        'FontSize', 14, 'FontWeight', 'bold');


fprintf('GPS sample rate: %.1f Hz\n', ...
        1/mean(diff(t_gps)));
fprintf('RCOU sample rate: %.1f Hz\n', ...
        1/mean(diff(t_rcou)));
fprintf('Max ground speed: %.2f m/s\n', max(gps_spd));
fprintf('Max throttle:     %.1f%%\n',   max(throttle_pct));


%% Yaw-Rate Rtep Response
%% USV Open-Loop Yaw Step Response
% System identification of yaw rate model
% Input:  RCOU_C2 — rudder command to servo (%)
% Output: IMU_GyrZ — yaw rate (rad/s converted to deg/s)

%% ── Time bases (zero-referenced) ─────────────────────────────────────────
t_rcou = d.RCOU_timestamp - t0;
t_imu  = d.IMU_timestamp  - t0;

%% ── Signals ──────────────────────────────────────────────────────────────
rudder_pct = d.RCOU_C2_pct_rudder;            % -100 to +100%
yaw_rate   = d.IMU_GyrZ * (180/pi);           % rad/s → deg/s

%% ── Plot ─────────────────────────────────────────────────────────────────
fig = figure('Name', 'USV Open-Loop Yaw Step Response', ...
             'Position', [100 100 1100 500]);

yyaxis left
plot(t_rcou, rudder_pct, 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'Rudder Command (%)');
ylabel('Rudder Command (%)');
ylim([-110 110]);
yline(0, 'b:', 'LineWidth', 0.75);

yyaxis right
plot(t_imu, yaw_rate, 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Yaw Rate (deg/s)');
ylabel('Yaw Rate (deg/s)');
yline(0, 'r:', 'LineWidth', 0.75);

xlabel('Time (s)');
title('Open-Loop Yaw Step Response — Rudder Command vs Yaw Rate');
legend('Location', 'best');
grid on;

sgtitle('Yaw System ID — Open Loop Step Response', ...
        'FontSize', 14, 'FontWeight', 'bold');