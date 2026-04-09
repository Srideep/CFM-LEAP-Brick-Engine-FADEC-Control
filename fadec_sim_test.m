%% FADEC_SIM_TEST.m
%  Complete closed-loop FADEC simulation — no hardware required.
%
%  This script runs the entire FADEC control loop in pure MATLAB:
%    Throttle input → TLA lookup → PID → Limit logic → Motor plant model
%         ↑                                                    |
%         └──── Thermo model ← N1 feedback ←──────────────────┘
%
%  Use this to develop, tune, and validate your FADEC while waiting
%  for the EV3 and CFM LEAP hardware to arrive.
%
%  Run: >> fadec_sim_test
%  Then open the generated plots to analyze system response.

clear; clc; close all;

%% ========================================================================
%  Load FADEC parameters
%  ========================================================================
fadec_setup;

%% ========================================================================
%  Simulation settings
%  ========================================================================
Ts = pid.Ts;                  % Sample time (0.02s = 50 Hz)
T_total = 60;                 % Total simulation time (seconds)
N_steps = round(T_total / Ts);

%% ========================================================================
%  Throttle profile — simulates pilot inputs over time
%  ========================================================================
%  Modify this to test different scenarios.
%
%  Format: [time_start, time_end, tla_degrees]
%  The throttle lever angle is interpolated between segments.

throttle_profile = [
%   t_start  t_end  tla_deg
    0        2      -30      % Engine at idle
    2        4       30      % Advance to climb
    4       12       30      % Hold climb
   12       14       90      % Advance toward TOGA
   14       22       90      % Hold near TOGA
   22       24       60      % Pull back to cruise
   24       35       60      % Hold cruise
   35       37      120      % Slam to full TOGA (test accel limiter)
   37       45      120      % Hold full TOGA (test EGT limiting)
   45       47      -30      % Snap to idle (test decel limiter)
   47       55      -30      % Hold idle
   55       57       90      % Quick advance (test accel limiter again)
   57       60       90      % Hold
];

% Build TLA time series from profile
tla_timeseries = zeros(N_steps, 1);
for step = 1:N_steps
    t = (step - 1) * Ts;
    tla_timeseries(step) = interp_throttle(t, throttle_profile);
end

%% ========================================================================
%  External load profile (disturbances)
%  ========================================================================
%  Simulates someone pressing on the fan, crosswind, etc.
%  Set to zero for nominal operation.

load_timeseries = zeros(N_steps, 1);
% Uncomment to add a load disturbance at t=30s:
% load_timeseries(round(30/Ts):round(33/Ts)) = 0.01;  % 0.01 N·m pulse

%% ========================================================================
%  Pre-allocate data logging arrays
%  ========================================================================
log.time       = zeros(N_steps, 1);
log.tla_deg    = zeros(N_steps, 1);
log.n1_demand  = zeros(N_steps, 1);
log.n1_actual  = zeros(N_steps, 1);
log.pid_output = zeros(N_steps, 1);
log.n1_limited = zeros(N_steps, 1);
log.pwm_cmd    = zeros(N_steps, 1);
log.egt        = zeros(N_steps, 1);
log.ps3        = zeros(N_steps, 1);
log.wf         = zeros(N_steps, 1);
log.fn         = zeros(N_steps, 1);
log.motor_i    = zeros(N_steps, 1);
log.accel_flag = zeros(N_steps, 1);
log.egt_flag   = zeros(N_steps, 1);
log.ovspd_flag = zeros(N_steps, 1);

%% ========================================================================
%  PID controller state
%  ========================================================================
pid_integral = 0;
pid_prev_error = 0;

%% ========================================================================
%  Main simulation loop
%  ========================================================================
fprintf('Running FADEC simulation: %.0f seconds at %.0f Hz...\n', T_total, 1/Ts);

for k = 1:N_steps
    t = (k - 1) * Ts;

    % --- 1. Read throttle lever angle ---
    tla_deg = tla_timeseries(k);

    % --- 2. TLA → N1 demand (1-D lookup) ---
    n1_demand = interp1(tla_schedule.lever_deg, tla_schedule.n1_demand, ...
                        tla_deg, 'linear', 'extrap');
    n1_demand = max(engine.N1_idle, min(engine.N1_max, n1_demand));

    % --- 3. Read N1 actual from plant model ---
    %     (On real hardware, this comes from the EV3 encoder)
    %     First iteration: use 0, then it feeds back from step 7.
    if k == 1
        n1_actual = 0;
    end

    % --- 4. PID controller ---
    error = n1_demand - n1_actual;

    pid_integral = pid_integral + error * Ts;
    % Anti-windup: clamp integral
    max_integral = pid.output_max / max(pid.Ki, 0.01);
    pid_integral = max(-max_integral, min(max_integral, pid_integral));

    if k > 1
        pid_derivative = (error - pid_prev_error) / Ts;
    else
        pid_derivative = 0;
    end

    pid_out = pid.Kp * error + pid.Ki * pid_integral + pid.Kd * pid_derivative;
    pid_out = max(pid.output_min, min(pid.output_max, pid_out));
    pid_prev_error = error;

    % --- 5. Thermodynamic model (uses previous N1) ---
    [egt_val, ps3_val, wf_val, fn_val] = fadec_thermo_model(n1_actual);

    % --- 6. Limit logic ---
    [n1_limited, accel_flag, egt_flag, ovspd_flag] = ...
        fadec_limit_logic(pid_out, n1_actual, egt_val, Ts);

    % --- 7. Motor plant model ---
    pwm_cmd = n1_limited;  % In sim, limited N1 maps directly to PWM
    ext_load = load_timeseries(k);

    [n1_actual, n1_pos, motor_i] = ev3_motor_plant(pwm_cmd, ext_load, Ts);

    % --- 8. Log everything ---
    log.time(k)       = t;
    log.tla_deg(k)    = tla_deg;
    log.n1_demand(k)  = n1_demand;
    log.n1_actual(k)  = n1_actual;
    log.pid_output(k) = pid_out;
    log.n1_limited(k) = n1_limited;
    log.pwm_cmd(k)    = pwm_cmd;
    log.egt(k)        = egt_val;
    log.ps3(k)        = ps3_val;
    log.wf(k)         = wf_val;
    log.fn(k)         = fn_val;
    log.motor_i(k)    = motor_i;
    log.accel_flag(k) = accel_flag;
    log.egt_flag(k)   = egt_flag;
    log.ovspd_flag(k) = ovspd_flag;
end

fprintf('Simulation complete.\n\n');

%% ========================================================================
%  Plot results — EICAS-style analysis
%  ========================================================================

figure('Name', 'FADEC Simulation Results', 'Position', [50 50 1200 900]);

% --- N1 speed ---
subplot(4, 2, 1);
plot(log.time, log.n1_demand, 'b--', 'LineWidth', 1.2); hold on;
plot(log.time, log.n1_actual, 'b-', 'LineWidth', 1.5);
plot(log.time, log.n1_limited, 'r:', 'LineWidth', 1);
yline(engine.N1_redline, 'r-', 'N1 Redline', 'LineWidth', 1.5);
yline(engine.N1_idle, 'k:', 'Idle');
ylabel('N1 (%)');
title('N1 Spool Speed');
legend('N1 demand', 'N1 actual', 'N1 limited', 'Location', 'best');
grid on; xlim([0 T_total]);

% --- EGT ---
subplot(4, 2, 2);
plot(log.time, log.egt, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5);
yline(engine.EGT_redline, 'r-', 'EGT Redline', 'LineWidth', 1.5);
yline(engine.EGT_redline - 30, 'r:', 'Margin');
ylabel('EGT (°C)');
title('Exhaust Gas Temperature');
grid on; xlim([0 T_total]);

% --- Thrust ---
subplot(4, 2, 3);
plot(log.time, log.fn, 'Color', [0.1 0.6 0.3], 'LineWidth', 1.5);
ylabel('Thrust (kN)');
title('Net Thrust');
grid on; xlim([0 T_total]);

% --- Fuel flow ---
subplot(4, 2, 4);
plot(log.time, log.wf, 'Color', [0.5 0.2 0.7], 'LineWidth', 1.5);
ylabel('Wf (kg/hr)');
title('Fuel Flow');
grid on; xlim([0 T_total]);

% --- Throttle lever angle ---
subplot(4, 2, 5);
plot(log.time, log.tla_deg, 'Color', [0.7 0.5 0.0], 'LineWidth', 1.5);
ylabel('TLA (deg)');
title('Throttle Lever Angle');
grid on; xlim([0 T_total]);

% --- PWM command ---
subplot(4, 2, 6);
plot(log.time, log.pwm_cmd, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
ylabel('PWM (%)');
title('Motor PWM Command');
grid on; xlim([0 T_total]);

% --- Limit flags ---
subplot(4, 2, 7);
area(log.time, log.accel_flag * 3, 'FaceColor', [1 0.8 0], 'EdgeColor', 'none', ...
    'FaceAlpha', 0.6); hold on;
area(log.time, log.egt_flag * 2, 'FaceColor', [1 0.4 0], 'EdgeColor', 'none', ...
    'FaceAlpha', 0.6);
area(log.time, log.ovspd_flag * 1, 'FaceColor', [1 0 0], 'EdgeColor', 'none', ...
    'FaceAlpha', 0.6);
ylabel('Protection');
title('Limit Flags');
legend('Accel limiter', 'EGT protection', 'Overspeed', 'Location', 'best');
yticks([0 1 2 3]);
yticklabels({'Off', 'Overspeed', 'EGT', 'Accel'});
grid on; xlim([0 T_total]);

% --- Compressor discharge pressure ---
subplot(4, 2, 8);
plot(log.time, log.ps3, 'Color', [0.3 0.6 0.6], 'LineWidth', 1.5);
ylabel('Ps3 (psia)');
title('Compressor Discharge Pressure');
grid on; xlim([0 T_total]);

xlabel('Time (s)');
sgtitle('FADEC Simulation — CFM LEAP Hobby Model', 'FontWeight', 'bold');

%% ========================================================================
%  Print summary statistics
%  ========================================================================
fprintf('============================================\n');
fprintf('  FADEC Simulation Summary\n');
fprintf('============================================\n');
fprintf('  Duration:          %.0f s\n', T_total);
fprintf('  Sample rate:       %.0f Hz\n', 1/Ts);
fprintf('  N1 max reached:    %.1f %%\n', max(log.n1_actual));
fprintf('  EGT max reached:   %.0f °C\n', max(log.egt));
fprintf('  Thrust max:        %.1f kN\n', max(log.fn));
fprintf('  Fuel flow max:     %.0f kg/hr\n', max(log.wf));
fprintf('  Accel limiter:     active %.1f %% of time\n', ...
    100 * sum(log.accel_flag) / N_steps);
fprintf('  EGT protection:    active %.1f %% of time\n', ...
    100 * sum(log.egt_flag) / N_steps);
fprintf('  Overspeed protect: active %.1f %% of time\n', ...
    100 * sum(log.ovspd_flag) / N_steps);
fprintf('============================================\n');
fprintf('\nPID Gains: Kp=%.2f  Ki=%.2f  Kd=%.2f\n', pid.Kp, pid.Ki, pid.Kd);
fprintf('Adjust gains in fadec_setup.m and re-run.\n');

%% ========================================================================
%  Helper function: interpolate throttle profile
%  ========================================================================
function tla = interp_throttle(t, profile)
    tla = profile(1, 3);  % Default to first value
    for row = 1:size(profile, 1)
        t_start = profile(row, 1);
        t_end   = profile(row, 2);
        tla_val = profile(row, 3);
        if t >= t_start && t < t_end
            % Check if previous segment exists for interpolation
            if row > 1 && t < t_start + 0.5
                % Brief ramp from previous segment
                prev_tla = profile(row - 1, 3);
                ramp_frac = (t - t_start) / 0.5;
                ramp_frac = max(0, min(1, ramp_frac));
                tla = prev_tla + (tla_val - prev_tla) * ramp_frac;
            else
                tla = tla_val;
            end
            return;
        end
    end
    % Past all segments: hold last value
    tla = profile(end, 3);
end
