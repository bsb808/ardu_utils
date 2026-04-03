
%% USV Throttle & Heading Analysis
% Uses only fields confirmed present: ATT, RCOU, RCIN, NTUN

clear; clc; close all;

fname = "/home/bsb/Downloads/00000008.BIN.mat"
d = load(fname);
fieldnames(d)

%% ── Configuration ────────────────────────────────────────────────────────
MAT_FILE         = "/home/bsb/Downloads/00000008.BIN.mat"
THROTTLE_CH_OUT  = 'RCOU_C1';
RUDDER_CH_OUT    = 'RCOU_C2';
THROTTLE_CH_IN   = 'RCIN_C1';
RUDDER_CH_IN     = 'RCIN_C2';

%% ── Load ─────────────────────────────────────────────────────────────────
fprintf('Loading %s...\n', MAT_FILE);
d = load(MAT_FILE);
fieldnames(d);

%% ── Time bases (zero-referenced) ─────────────────────────────────────────
t_att  = d.ATT_timestamp  - d.ATT_timestamp(1);
t_rcou = d.RCOU_timestamp - d.ATT_timestamp(1);
t_rcin = d.RCIN_timestamp - d.ATT_timestamp(1);
t_ntun = d.NTUN_timestamp - d.ATT_timestamp(1);

% Common time base — use ATT as reference
t_ref = t_att;

%% ── Interpolate onto common time base ───────────────────────────────────
throttle_pwm_out = interp1(t_rcou, d.(THROTTLE_CH_OUT),          t_ref, 'linear', 'extrap');
throttle_pct_out = interp1(t_rcou, d.RCOU_C1_pct_throttle,       t_ref, 'linear', 'extrap');
throttle_pwm_in  = interp1(t_rcin, d.(THROTTLE_CH_IN),           t_ref, 'linear', 'extrap');
throttle_pct_in  = interp1(t_rcin, d.RCIN_C1_pct_throttle,       t_ref, 'linear', 'extrap');

rudder_pwm_out   = interp1(t_rcou, d.(RUDDER_CH_OUT),            t_ref, 'linear', 'extrap');
rudder_pct_out   = interp1(t_rcou, d.RCOU_C2_pct_rudder,         t_ref, 'linear', 'extrap');
rudder_pwm_in    = interp1(t_rcin, d.(RUDDER_CH_IN),             t_ref, 'linear', 'extrap');
rudder_pct_in    = interp1(t_rcin, d.RCIN_C2_pct_rudder,         t_ref, 'linear', 'extrap');

des_yaw          = interp1(t_att,  d.ATT_DesYaw,                 t_ref, 'linear', 'extrap');
act_yaw          = interp1(t_att,  d.ATT_Yaw,                    t_ref, 'linear', 'extrap');
ntun_des_yaw     = interp1(t_ntun, d.NTUN_DesYaw,                t_ref, 'linear', 'extrap');

%% ── Detect throttle steps ────────────────────────────────────────────────
STEP_THRESHOLD = 5;  % percent
d_throttle = [0, diff(throttle_pct_out)];
step_idx   = find(abs(d_throttle) > STEP_THRESHOLD);

%% ── Figure 1: Throttle ───────────────────────────────────────────────────
fig1 = figure('Name', 'Throttle Analysis', 'Position', [100 100 1000 700]);

ax1 = subplot(3,1,1);
plot(t_ref, throttle_pwm_in,  'r--', 'LineWidth', 1.2, 'DisplayName', 'RC Input'); hold on;
plot(t_ref, throttle_pwm_out, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Autopilot Output');
yline(1500, 'k--', 'LineWidth', 0.75);
ylabel('PWM (\mus)'); title('Throttle — PWM');
ylim([900 2100]); legend('Location','best'); grid on;
for i = 1:length(step_idx)
    xline(t_ref(step_idx(i)), 'r:', 'Alpha', 0.4);
end

ax2 = subplot(3,1,2);
plot(t_ref, throttle_pct_in,  'r--', 'LineWidth', 1.2, 'DisplayName', 'RC Input'); hold on;
plot(t_ref, throttle_pct_out, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Autopilot Output');
yline(0,   'k--', 'LineWidth', 0.75);
yline(100, 'k:',  'LineWidth', 0.75);
ylabel('Effort (%)'); title('Throttle — Percent Effort');
ylim([-5 110]); legend('Location','best'); grid on;
for i = 1:length(step_idx)
    xline(t_ref(step_idx(i)), 'r:', 'Alpha', 0.4);
end

ax3 = subplot(3,1,3);
% Note: without CTUN, actual vessel speed is not available in this log
% Heading used as a proxy for vessel state until CTUN logging is enabled
plot(t_ref, act_yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Heading');
ylabel('Heading (deg)'); xlabel('Time (s)');
title('Vessel Heading (speed unavailable — enable CTUN logging)');
legend('Location','best'); grid on;

linkaxes([ax1 ax2 ax3], 'x');
sgtitle('USV Throttle Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Figure 2: Rudder & Heading ───────────────────────────────────────────
fig2 = figure('Name', 'Rudder & Heading Analysis', 'Position', [150 150 1000 700]);

ax4 = subplot(3,1,1);
plot(t_ref, rudder_pwm_in,  'r--', 'LineWidth', 1.2, 'DisplayName', 'RC Input'); hold on;
plot(t_ref, rudder_pwm_out, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Autopilot Output');
yline(1500, 'k--', 'LineWidth', 0.75, 'Label', 'Neutral');
ylabel('PWM (\mus)'); title('Rudder — PWM');
ylim([900 2100]); legend('Location','best'); grid on;

ax5 = subplot(3,1,2);
plot(t_ref, rudder_pct_in,  'r--', 'LineWidth', 1.2, 'DisplayName', 'RC Input'); hold on;
plot(t_ref, rudder_pct_out, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Autopilot Output');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Effort (%)'); title('Rudder — Percent Effort');
ylim([-110 110]); legend('Location','best'); grid on;

ax6 = subplot(3,1,3);
plot(t_ref, des_yaw,     'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired Yaw (ATT)'); hold on;
plot(t_ref, ntun_des_yaw,'g--', 'LineWidth', 1.2, 'DisplayName', 'Desired Yaw (NTUN)');
plot(t_ref, act_yaw,     'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual Yaw');
ylabel('Heading (deg)'); xlabel('Time (s)');
title('Heading — Setpoint vs Actual');
legend('Location','best'); grid on;

linkaxes([ax4 ax5 ax6], 'x');
sgtitle('USV Rudder & Heading Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Export ───────────────────────────────────────────────────────────────
% exportgraphics(fig1, 'throttle_analysis.pdf', 'ContentType', 'vector');
% exportgraphics(fig2, 'rudder_heading_analysis.pdf', 'ContentType', 'vector');
% fprintf('Figures saved.\n');

%% ── Log parameter recommendations ───────────────────────────────────────
fprintf('\n--- Enable these in LOG_BITMASK for full PID analysis ---\n');
fprintf('  CTUN  (bit 2)  — speed setpoint and actual speed\n');
fprintf('  PID   (bit 12) — PIDY, PIDX PID term logging\n');
fprintf('  RATE  (bit 7)  — rate controller inputs and outputs\n');

%% ── Figure 3: PIDS — Steering PID ────────────────────────────────────────
t_pids = d.PIDS_timestamp - d.ATT_timestamp(1);

% Interpolate onto common time base
pids_tar  = interp1(t_pids, d.PIDS_Tar,  t_ref, 'linear', 'extrap');
pids_act  = interp1(t_pids, d.PIDS_Act,  t_ref, 'linear', 'extrap');
pids_err  = interp1(t_pids, d.PIDS_Err,  t_ref, 'linear', 'extrap');
pids_P    = interp1(t_pids, d.PIDS_P,    t_ref, 'linear', 'extrap');
pids_I    = interp1(t_pids, d.PIDS_I,    t_ref, 'linear', 'extrap');
pids_D    = interp1(t_pids, d.PIDS_D,    t_ref, 'linear', 'extrap');
pids_FF   = interp1(t_pids, d.PIDS_FF,   t_ref, 'linear', 'extrap');
pids_out  = pids_P + pids_I + pids_D + pids_FF;

fig3 = figure('Name', 'PIDS — Steering PID', 'Position', [200 200 1000 800]);

ax7 = subplot(4,1,1);
plot(t_ref, pids_tar, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
plot(t_ref, pids_act, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
ylabel('Steering Demand'); title('PIDS — Target vs Actual');
legend('Location', 'best'); grid on;

ax8 = subplot(4,1,2);
plot(t_ref, pids_err, 'k-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Error'); title('PIDS — Error');
grid on;

ax9 = subplot(4,1,3);
plot(t_ref, pids_P,  'b-',  'LineWidth', 1.5, 'DisplayName', 'P'); hold on;
plot(t_ref, pids_I,  'r-',  'LineWidth', 1.5, 'DisplayName', 'I');
plot(t_ref, pids_D,  'g-',  'LineWidth', 1.5, 'DisplayName', 'D');
plot(t_ref, pids_FF, 'm--', 'LineWidth', 1.2, 'DisplayName', 'FF');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('PID Terms'); title('PIDS — P, I, D, FF Terms');
legend('Location', 'best'); grid on;

ax10 = subplot(4,1,4);
plot(t_ref, pids_out,        'b-',  'LineWidth', 1.5, 'DisplayName', 'PID Output'); hold on;
plot(t_ref, rudder_pct_out,  'r--', 'LineWidth', 1.5, 'DisplayName', 'Rudder Output (%)');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Output'); xlabel('Time (s)');
title('PIDS — Total Output vs Rudder Command');
legend('Location', 'best'); grid on;

linkaxes([ax7 ax8 ax9 ax10], 'x');
sgtitle('Steering PID (PIDS) Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Figure 4: PIDA — Altitude PID (diagnostic only) ─────────────────────
t_pida = d.PIDA_timestamp - d.ATT_timestamp(1);

pida_tar  = interp1(t_pida, d.PIDA_Tar, t_ref, 'linear', 'extrap');
pida_act  = interp1(t_pida, d.PIDA_Act, t_ref, 'linear', 'extrap');
pida_err  = interp1(t_pida, d.PIDA_Err, t_ref, 'linear', 'extrap');
pida_P    = interp1(t_pida, d.PIDA_P,   t_ref, 'linear', 'extrap');
pida_I    = interp1(t_pida, d.PIDA_I,   t_ref, 'linear', 'extrap');
pida_D    = interp1(t_pida, d.PIDA_D,   t_ref, 'linear', 'extrap');
pida_FF   = interp1(t_pida, d.PIDA_FF,  t_ref, 'linear', 'extrap');

fig4 = figure('Name', 'PIDA — Altitude PID', 'Position', [250 250 1000 800]);

ax11 = subplot(4,1,1);
plot(t_ref, pida_tar, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
plot(t_ref, pida_act, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
ylabel('Altitude Demand'); title('PIDA — Target vs Actual (diagnostic — not active on USV)');
legend('Location', 'best'); grid on;

ax12 = subplot(4,1,2);
plot(t_ref, pida_err, 'k-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Error'); title('PIDA — Error');
grid on;

ax13 = subplot(4,1,3);
plot(t_ref, pida_P,  'b-',  'LineWidth', 1.5, 'DisplayName', 'P'); hold on;
plot(t_ref, pida_I,  'r-',  'LineWidth', 1.5, 'DisplayName', 'I');
plot(t_ref, pida_D,  'g-',  'LineWidth', 1.5, 'DisplayName', 'D');
plot(t_ref, pida_FF, 'm--', 'LineWidth', 1.2, 'DisplayName', 'FF');
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('PID Terms'); title('PIDA — P, I, D, FF Terms');
legend('Location', 'best'); grid on;

ax14 = subplot(4,1,4);
pida_out = pida_P + pida_I + pida_D + pida_FF;
plot(t_ref, pida_out, 'b-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.75);
ylabel('Output'); xlabel('Time (s)');
title('PIDA — Total Output');
grid on;

linkaxes([ax11 ax12 ax13 ax14], 'x');
sgtitle('Altitude PID (PIDA) — Diagnostic Only', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Export ───────────────────────────────────────────────────────────────
% exportgraphics(fig3, 'pids_steering_pid.pdf', 'ContentType', 'vector');
% exportgraphics(fig4, 'pida_altitude_pid.pdf', 'ContentType', 'vector');
fprintf('PID figures saved.\n');