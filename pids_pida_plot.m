%% USV PID Analysis
% Figure 1: PIDS — Steering PID
% Figure 2: PIDA — Altitude/Yaw PID

clear; clc; close all;

%% ── Load ─────────────────────────────────────────────────────────────────
fname = "/home/bsb/Downloads/00000008.BIN.mat";
fprintf('Loading %s...\n', fname);
d = load(fname);

%% ── Time bases (zero-referenced to first ATT message) ───────────────────
t0 = d.ATT_timestamp(1);

t_pids = d.PIDS_timestamp - t0;
t_pida = d.PIDA_timestamp - t0;

%% ── Figure 1: PIDS — Steering PID ───────────────────────────────────────
fig1 = figure('Name', 'PIDS — Steering PID', 'Position', [100 100 1000 800]);

ax1 = subplot(3,1,1);
plot(t_pids, d.PIDS_Tar, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
plot(t_pids, d.PIDS_Act, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
ylabel('Steering Rate (deg/s)');
title('PIDS — Steering Target vs Actual');
legend('Location', 'best'); grid on;

ax2 = subplot(3,1,2);
plot(t_pids, d.PIDS_Err, 'k-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Error (deg/s)');
title('PIDS — Steering Error');
grid on;

ax3 = subplot(3,1,3);
plot(t_pids, d.PIDS_P,  'b-',  'LineWidth', 1.5, 'DisplayName', 'P'); hold on;
plot(t_pids, d.PIDS_I,  'r-',  'LineWidth', 1.5, 'DisplayName', 'I');
plot(t_pids, d.PIDS_D,  'g-',  'LineWidth', 1.5, 'DisplayName', 'D');
plot(t_pids, d.PIDS_FF, 'm--', 'LineWidth', 1.2, 'DisplayName', 'FF');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('PID Output (normalized)');
xlabel('Time (s)');
title('PIDS — P, I, D, FF Terms');
legend('Location', 'best'); grid on;

linkaxes([ax1 ax2 ax3], 'x');
sgtitle('Steering PID (PIDS)', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Figure 2: PIDA — Yaw/Altitude PID ───────────────────────────────────
fig2 = figure('Name', 'PIDA — Yaw PID', 'Position', [150 150 1000 800]);

ax4 = subplot(3,1,1);
plot(t_pida, d.PIDA_Tar, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
plot(t_pida, d.PIDA_Act, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
ylabel('Yaw Rate (deg/s)');
title('PIDA — Yaw Target vs Actual');
legend('Location', 'best'); grid on;

ax5 = subplot(3,1,2);
plot(t_pida, d.PIDA_Err, 'k-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Error (deg/s)');
title('PIDA — Yaw Error');
grid on;

ax6 = subplot(3,1,3);
plot(t_pida, d.PIDA_P,  'b-',  'LineWidth', 1.5, 'DisplayName', 'P'); hold on;
plot(t_pida, d.PIDA_I,  'r-',  'LineWidth', 1.5, 'DisplayName', 'I');
plot(t_pida, d.PIDA_D,  'g-',  'LineWidth', 1.5, 'DisplayName', 'D');
plot(t_pida, d.PIDA_FF, 'm--', 'LineWidth', 1.2, 'DisplayName', 'FF');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('PID Output (normalized)');
xlabel('Time (s)');
title('PIDA — P, I, D, FF Terms');
legend('Location', 'best'); grid on;

linkaxes([ax4 ax5 ax6], 'x');
sgtitle('Yaw PID (PIDA)', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Export ───────────────────────────────────────────────────────────────
exportgraphics(fig1, 'pids_steering.pdf', 'ContentType', 'vector');
exportgraphics(fig2, 'pida_yaw.pdf',      'ContentType', 'vector');
fprintf('Figures saved.\n');