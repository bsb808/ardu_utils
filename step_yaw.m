%% USV Open-Loop Yaw Step Response
% Input:  RCIN_C2 — rudder stick command (%)
% Output: ATT_Yaw differentiated → yaw rate (deg/s)

clear; clc; close all;

%% ── Load ─────────────────────────────────────────────────────────────────
fname = "/home/bsb/Downloads/00000008.BIN.mat";
fprintf('Loading %s...\n', fname);
d = load(fname);

%% ── Time bases (zero-referenced) ─────────────────────────────────────────
t0     = d.ATT_timestamp(1);
t_att  = d.ATT_timestamp - t0;
t_rcin = d.RCIN_timestamp - t0;

%% ── Rudder command ───────────────────────────────────────────────────────
rudder_cmd = d.RCIN_C2_pct_rudder;   % % effort (-100 to +100)

%% ── Yaw rate from ATT_Yaw ────────────────────────────────────────────────
% Unwrap yaw to handle 0/360 boundary crossing
yaw_deg      = unwrap(d.ATT_Yaw * pi/180) * 180/pi;

% Numerical differentiation
dt           = diff(t_att);
yaw_rate_raw = diff(yaw_deg) ./ dt;   % deg/s
t_rate       = t_att(1:end-1);        % time vector for rate

% Low-pass filter to reduce numerical noise
% Cutoff at 2 Hz — adjust as needed for your data
fs           = 1 / mean(dt);          % approximate sample rate
fc           = 2.0;                   % cutoff frequency (Hz)
[b, a]       = butter(2, fc / (fs/2), 'low');
yaw_rate_filt = filtfilt(b, a, yaw_rate_raw);

%% ── Plot ─────────────────────────────────────────────────────────────────
fig = figure('Name', 'USV Open-Loop Yaw Step Response', ...
             'Position', [100 100 1100 500]);

yyaxis left
plot(t_rcin, rudder_cmd, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Rudder Command (%)');
ylabel('Rudder Command (%)');
ylim([-110 110]);
yline(0, 'b:', 'LineWidth', 0.75);

yyaxis right
plot(t_rate, yaw_rate_raw,  'r-',  'LineWidth', 0.75, 'DisplayName', 'Yaw Rate Raw (deg/s)'); hold on;
plot(t_rate, yaw_rate_filt, 'g-',  'LineWidth', 2.0,  'DisplayName', 'Yaw Rate Filtered (deg/s)');
ylabel('Yaw Rate (deg/s)');
yline(0, 'k:', 'LineWidth', 0.75);

xlabel('Time (s)');
title('Open-Loop Yaw Step Response — Rudder Command vs Yaw Rate');
legend('Location', 'best');
grid on;

sgtitle('Open-Loop Yaw Step Response', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Export ───────────────────────────────────────────────────────────────
exportgraphics(fig, 'yaw_step_response_openloop.pdf', 'ContentType', 'vector');
fprintf('Figure saved: yaw_step_response_openloop.pdf\n');
fprintf('ATT sample rate: %.1f Hz\n', fs);
fprintf('Filter cutoff:   %.1f Hz\n', fc);