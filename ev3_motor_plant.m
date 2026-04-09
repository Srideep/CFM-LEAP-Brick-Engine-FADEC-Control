function [n1_speed_pct, n1_position_deg, motor_current] = ...
    ev3_motor_plant(pwm_command, external_load, Ts)
%EV3_MOTOR_PLANT  Simulated EV3 Large Motor + CFM LEAP fan load.
%
%   Models the physical plant that the FADEC controls: a DC motor
%   driving a fan through a gear train, with realistic dynamics.
%
%   This lets you develop and tune the entire FADEC control law
%   in pure Simulink before any hardware arrives.
%
%   Motor model: armature-controlled DC motor
%     V_applied = R*i + L*(di/dt) + Ke*omega
%     T_motor   = Kt * i
%     J*(domega/dt) = T_motor - B*omega - T_load
%
%   Simplified to first-order (L/R << mechanical time constant):
%     T_motor = Kt/R * (V - Ke*omega)
%     J*(domega/dt) = T_motor - B*omega - T_load
%
%   Inputs:
%       pwm_command    - Motor command from FADEC (-100 to +100 %)
%       external_load  - External disturbance torque (N·m)
%                        Simulate finger on fan, wind, etc.
%       Ts             - Sample time (seconds), typically 0.02
%
%   Outputs:
%       n1_speed_pct   - Fan spool speed as percentage (0-100%)
%                        Maps to N1 on the FADEC gauges
%       n1_position_deg - Cumulative position (degrees)
%                         Matches EV3 encoder output
%       motor_current  - Armature current (A), for diagnostics
%
%   Usage in Simulink:
%       1. Add a MATLAB Function block
%       2. Paste this function
%       3. Connect pwm_command from your FADEC limit logic output
%       4. Connect external_load to a Constant (0) or Slider
%       5. Feed n1_speed_pct back to your PID as the feedback signal
%       6. Feed n1_speed_pct to fadec_thermo_model for EGT etc.
%
%   When hardware arrives, REPLACE this block with the real
%   EV3 Motor + Encoder blocks. Your FADEC logic stays identical.

    % =====================================================================
    % EV3 Large Motor parameters (45502)
    % Measured/estimated from public teardown data
    % =====================================================================

    % Electrical
    V_nominal = 9.0;         % Nominal battery voltage (V) — 6x AA
    R_armature = 6.2;        % Armature resistance (ohms)
    Ke = 0.0085;             % Back-EMF constant (V·s/rad)
    Kt = 0.0085;             % Torque constant (N·m/A) — Kt ≈ Ke for DC motor

    % Mechanical
    J_motor = 0.8e-4;        % Motor rotor inertia (kg·m²)
    J_fan = 3.0e-4;          % Fan + gear train equivalent inertia (kg·m²)
    J_total = J_motor + J_fan;
    B_viscous = 0.0003;      % Viscous friction coefficient (N·m·s/rad)
    T_coulomb = 0.003;       % Coulomb (static) friction torque (N·m)

    % Gear ratio: motor → fan shaft
    % Typical Technic gear train: 12t → 20t or similar
    gear_ratio = 1.667;      % Motor turns / fan turns

    % Speed scaling
    % EV3 Large Motor no-load speed ≈ 170 RPM at 9V
    % We map this to ~100% N1
    omega_max_rpm = 170;
    omega_max_rad = omega_max_rpm * 2 * pi / 60;  % ~17.8 rad/s

    % =====================================================================
    % State variables (persistent across time steps)
    % =====================================================================
    persistent omega;        % Angular velocity (rad/s) at motor shaft
    persistent theta;        % Cumulative position (rad) at motor shaft
    persistent i_armature;   % Armature current (A)

    if isempty(omega)
        omega = 0;
        theta = 0;
        i_armature = 0;
    end

    % =====================================================================
    % Input processing
    % =====================================================================

    % Clamp PWM to valid range
    pwm = max(-100, min(100, pwm_command));

    % Convert PWM percentage to applied voltage
    V_applied = (pwm / 100) * V_nominal;

    % =====================================================================
    % Motor electrical model (simplified: L negligible)
    % =====================================================================

    % Back-EMF
    V_bemf = Ke * omega;

    % Armature current
    i_armature = (V_applied - V_bemf) / R_armature;

    % Motor electromagnetic torque
    T_motor = Kt * i_armature;

    % =====================================================================
    % Mechanical model
    % =====================================================================

    % Coulomb friction (opposes motion)
    if abs(omega) > 0.01
        T_friction = B_viscous * omega + T_coulomb * sign(omega);
    else
        % Static friction regime — motor must overcome stiction
        T_friction = B_viscous * omega;
        if abs(T_motor) < T_coulomb
            T_friction = T_motor;  % Motor can't overcome stiction
        end
    end

    % Fan aerodynamic load (increases with speed squared)
    % This simulates the increasing air resistance on the fan blades
    K_aero = 1.5e-5;  % Aerodynamic drag coefficient
    T_aero = K_aero * omega * abs(omega);

    % Net torque on the shaft
    T_net = T_motor - T_friction - T_aero - external_load;

    % Angular acceleration
    alpha = T_net / J_total;

    % Integrate: velocity and position (Euler forward)
    omega = omega + alpha * Ts;

    % Prevent reverse rotation when PWM is positive (one-way bearing analog)
    % Comment this out if you want full bidirectional for reverse thrust
    % if pwm >= 0 && omega < 0
    %     omega = 0;
    % end

    theta = theta + omega * Ts;

    % =====================================================================
    % Output scaling
    % =====================================================================

    % Speed as N1 percentage (0-100%)
    % Motor shaft speed → fan speed via gear ratio → scale to %
    fan_omega = omega / gear_ratio;
    fan_rpm = fan_omega * 60 / (2 * pi);
    fan_max_rpm = omega_max_rpm / gear_ratio;

    n1_speed_pct = (fan_rpm / fan_max_rpm) * 100;
    n1_speed_pct = max(0, min(110, n1_speed_pct));  % Allow slight overshoot

    % Position in degrees (matches EV3 encoder output: 1 degree resolution)
    n1_position_deg = theta * (180 / pi) / gear_ratio;

    % Current for diagnostics
    motor_current = i_armature;

end
