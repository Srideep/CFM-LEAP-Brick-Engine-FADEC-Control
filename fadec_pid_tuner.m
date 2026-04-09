%% FADEC_PID_TUNER.m
%  Interactive PID gain exploration tool.
%
%  Runs a quick step-response test (idle → 80% N1) for each set of
%  PID gains and plots key performance metrics:
%    - Rise time (10% → 90%)
%    - Overshoot (%)
%    - Settling time (within 2% band)
%    - Steady-state error
%
%  Modify the gains_to_test matrix below and re-run to compare.
%  Once you find good gains, update fadec_setup.m.
%
%  Run: >> fadec_pid_tuner

clear; clc; close all;

%% Load base parameters
fadec_setup;

%% ========================================================================
%  Define gain sets to compare
%  ========================================================================
%  Each row: [Kp, Ki, Kd, label]

gains_to_test = {
    1.0,  0.3,  0.05,  'Conservative';
    2.0,  0.8,  0.10,  'Baseline (fadec_setup)';
    3.5,  1.5,  0.15,  'Aggressive';
    2.0,  1.2,  0.30,  'High derivative';
    6.0,  0.5,  0.05,  'High proportional';
};

N_tests = size(gains_to_test, 1);

%% ========================================================================
%  Test parameters
%  ========================================================================
Ts = 0.02;
T_test = 10;             % 10 second step response
N_steps = round(T_test / Ts);
t = (0:N_steps-1)' * Ts;

% Step: idle (19%) → target (80%)
n1_start = 19;
n1_target = 80;

%% ========================================================================
%  Run each gain set
%  ========================================================================
figure('Name', 'PID Gain Comparison', 'Position', [50 50 1100 700]);

colors = lines(N_tests);
metrics = zeros(N_tests, 4);  % rise_time, overshoot, settling, ss_error

for g = 1:N_tests
    Kp = gains_to_test{g, 1};
    Ki = gains_to_test{g, 2};
    Kd = gains_to_test{g, 3};
    label = gains_to_test{g, 4};

    % Reset plant model persistent variables
    clear ev3_motor_plant;

    % PID state
    integral = 0;
    prev_error = 0;
    n1_actual = n1_start;

    % Pre-spin plant to idle steady state
    for warmup = 1:200
        [n1_actual, ~, ~] = ev3_motor_plant(n1_start, 0, Ts);
    end

    % Run step response
    n1_log = zeros(N_steps, 1);

    for k = 1:N_steps
        error = n1_target - n1_actual;

        integral = integral + error * Ts;
        max_int = 100 / max(Ki, 0.01);
        integral = max(-max_int, min(max_int, integral));

        if k > 1
            derivative = (error - prev_error) / Ts;
        else
            derivative = 0;
        end

        pwm = Kp * error + Ki * integral + Kd * derivative;
        pwm = max(0, min(100, pwm));
        prev_error = error;

        [n1_actual, ~, ~] = ev3_motor_plant(pwm, 0, Ts);
        n1_log(k) = n1_actual;
    end

    % --- Compute metrics ---
    step_size = n1_target - n1_start;

    % Rise time (10% to 90% of step)
    thresh_10 = n1_start + 0.1 * step_size;
    thresh_90 = n1_start + 0.9 * step_size;
    idx_10 = find(n1_log >= thresh_10, 1, 'first');
    idx_90 = find(n1_log >= thresh_90, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90)
        rise_time = (idx_90 - idx_10) * Ts;
    else
        rise_time = NaN;
    end

    % Overshoot
    peak = max(n1_log);
    overshoot = max(0, (peak - n1_target) / step_size * 100);

    % Settling time (within 2% band)
    band = 0.02 * step_size;
    settled = abs(n1_log - n1_target) < band;
    % Find last time it leaves the band
    outside = find(~settled, 1, 'last');
    if isempty(outside)
        settling_time = 0;
    elseif outside < N_steps
        settling_time = outside * Ts;
    else
        settling_time = NaN;
    end

    % Steady-state error (average of last 1 second)
    ss_samples = round(1/Ts);
    ss_error = abs(n1_target - mean(n1_log(end-ss_samples:end)));

    metrics(g, :) = [rise_time, overshoot, settling_time, ss_error];

    % --- Plot ---
    subplot(2, 1, 1);
    plot(t, n1_log, 'Color', colors(g,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s (Kp=%.1f Ki=%.1f Kd=%.2f)', ...
        label, Kp, Ki, Kd));
    hold on;
end

% Finalize N1 plot
subplot(2, 1, 1);
yline(n1_target, 'k--', 'Target', 'LineWidth', 1);
yline(n1_start, 'k:', 'Idle');
yline(n1_target * 1.02, 'r:', '+2%');
yline(n1_target * 0.98, 'r:', '-2%');
ylabel('N1 (%)');
title('Step Response: Idle → 80% N1');
legend('Location', 'southeast', 'FontSize', 8);
grid on;
xlim([0 T_test]);
ylim([0 100]);

% Metrics bar chart
subplot(2, 1, 2);
metric_names = {'Rise time (s)', 'Overshoot (%)', 'Settling time (s)', 'SS error (%)'};
bar_data = metrics;
bar_data(isnan(bar_data)) = 0;

b = bar(bar_data);
for i = 1:size(bar_data, 2)
    b(i).FaceColor = 'flat';
end

set(gca, 'XTickLabel', cellfun(@(x) x, gains_to_test(:,4), 'UniformOutput', false));
xtickangle(30);
legend(metric_names, 'Location', 'northeast');
title('Performance Metrics Comparison');
ylabel('Value');
grid on;

%% ========================================================================
%  Print results table
%  ========================================================================
fprintf('\n');
fprintf('=======================================================================\n');
fprintf('  PID Gain Comparison Results\n');
fprintf('=======================================================================\n');
fprintf('  %-22s  %8s  %10s  %12s  %10s\n', ...
    'Gain Set', 'Rise(s)', 'Overshoot', 'Settle(s)', 'SS Err');
fprintf('-----------------------------------------------------------------------\n');

for g = 1:N_tests
    label = gains_to_test{g, 4};
    fprintf('  %-22s  %8.2f  %9.1f%%  %11.2f  %9.2f%%\n', ...
        label, metrics(g,1), metrics(g,2), metrics(g,3), metrics(g,4));
end

fprintf('=======================================================================\n');
fprintf('\nChoose the gain set with the best tradeoff between\n');
fprintf('fast rise time and low overshoot. For a FADEC, low overshoot\n');
fprintf('is more important than fast response (prevents N1 exceedance).\n');
fprintf('\nUpdate your chosen gains in fadec_setup.m:\n');
fprintf('  pid.Kp = ...; pid.Ki = ...; pid.Kd = ...;\n');
fprintf('\nThen re-run fadec_sim_test.m for the full scenario validation.\n');
