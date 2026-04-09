function [n1_cmd, start_state, starter_torque, fuel_valve] = ...
    engine_start_sequence(start_button, n1_actual, egt, Ts)
%ENGINE_START_SEQUENCE  CFM LEAP-style engine start state machine.
%
%   Models the startup sequence: OFF → CRANKING → LIGHTOFF → STABILIZE → IDLE
%   Produces a starter motor torque command and fuel valve signal.
%   The FADEC PID controller only takes over after reaching IDLE.
%
%   Inputs:
%       start_button  - 1 = start commanded (e.g. EV3 Touch Sensor)
%       n1_actual     - Current N1 from plant model (%)
%       egt           - Current EGT from thermo model (°C)
%       Ts            - Sample time (seconds)
%
%   Outputs:
%       n1_cmd        - N1 command to motor (%, 0 during start, idle after)
%       start_state   - Current state (0=OFF,1=CRANK,2=LIGHT,3=STAB,4=IDLE)
%       starter_torque - Direct motor PWM during start (bypasses PID)
%       fuel_valve    - 0 = closed, 1 = open
%
%   Usage:
%       During start_state < 4, route starter_torque directly to the
%       motor plant (bypass PID). Once start_state == 4, switch to
%       PID output for normal FADEC operation.

    % States
    OFF       = 0;
    CRANKING   = 1;
    LIGHTOFF   = 2;
    STABILIZE  = 3;
    IDLE       = 4;

    % Thresholds
    N1_CRANK_TARGET = 12;    % N1% to reach before fuel introduction
    N1_LIGHTOFF_EGT = 725;   % Max EGT during start (°C)
    N1_IDLE_TARGET  = 19;    % Idle N1%
    N1_IDLE_BAND    = 1.0;   % Within ±1% = stable at idle

    % Timing
    CRANK_RAMP_RATE   = 6;   % PWM %/sec ramp rate during cranking
    MAX_CRANK_TIME    = 10;  % Abort start if no lightoff after 10s
    STABILIZE_TIME    = 3;   % Seconds to stabilize before declaring idle

    % Persistent state
    persistent state;
    persistent timer;
    persistent crank_pwm;

    if isempty(state)
        state = OFF;
        timer = 0;
        crank_pwm = 0;
    end

    % Default outputs
    n1_cmd = 0;
    starter_torque = 0;
    fuel_valve = 0;

    switch state
        case OFF
            % Waiting for start command
            if start_button == 1
                state = CRANKING;
                timer = 0;
                crank_pwm = 5;  % Initial PWM to overcome stiction
                fprintf('[START] Cranking initiated\n');
            end
            n1_cmd = 0;
            starter_torque = 0;
            fuel_valve = 0;

        case CRANKING
            % Ramp motor slowly to simulate starter motor
            timer = timer + Ts;
            crank_pwm = crank_pwm + CRANK_RAMP_RATE * Ts;
            crank_pwm = min(crank_pwm, 30);  % Max starter PWM

            starter_torque = crank_pwm;
            fuel_valve = 0;  % No fuel yet

            % Transition: N1 reaches crank target → introduce fuel
            if n1_actual >= N1_CRANK_TARGET
                state = LIGHTOFF;
                timer = 0;
                fprintf('[START] N1 at %.1f%% — fuel on, lightoff\n', n1_actual);
            end

            % Timeout protection
            if timer > MAX_CRANK_TIME
                state = OFF;
                crank_pwm = 0;
                fprintf('[START] ABORT — crank timeout\n');
            end

        case LIGHTOFF
            % Fuel introduced — EGT rises sharply
            timer = timer + Ts;
            fuel_valve = 1;

            % Continue cranking but start reducing as combustion helps
            starter_torque = crank_pwm * max(0, 1 - timer/3);

            % EGT exceedance protection during start
            if egt > N1_LIGHTOFF_EGT
                % Hot start — abort
                state = OFF;
                crank_pwm = 0;
                fprintf('[START] ABORT — start EGT limit exceeded (%.0f°C)\n', egt);
                fuel_valve = 0;
                starter_torque = 0;
                return;
            end

            % N1 is self-sustaining when it exceeds lightoff + margin
            if n1_actual >= N1_CRANK_TARGET + 3
                state = STABILIZE;
                timer = 0;
                fprintf('[START] Self-sustaining at N1=%.1f%% — stabilizing\n', n1_actual);
            end

        case STABILIZE
            % Engine is running, let it settle to idle
            timer = timer + Ts;
            fuel_valve = 1;
            starter_torque = 0;  % Starter disengaged

            % Command idle N1 — PID should take over here
            n1_cmd = N1_IDLE_TARGET;

            % Check if N1 has stabilized near idle
            if timer > STABILIZE_TIME && ...
               abs(n1_actual - N1_IDLE_TARGET) < N1_IDLE_BAND
                state = IDLE;
                fprintf('[START] Idle stable at N1=%.1f%% — start complete\n', n1_actual);
            end

            % Timeout: if it takes too long, still declare idle
            if timer > STABILIZE_TIME * 3
                state = IDLE;
                fprintf('[START] Idle timeout — declaring idle at N1=%.1f%%\n', n1_actual);
            end

        case IDLE
            % Normal operation — FADEC PID has full control
            fuel_valve = 1;
            starter_torque = 0;
            n1_cmd = N1_IDLE_TARGET;  % Minimum command

            % If N1 drops to 0 (shutdown), return to OFF
            if n1_actual < 2
                state = OFF;
                timer = 0;
                crank_pwm = 0;
                fprintf('[START] Engine shutdown detected\n');
            end
    end

    start_state = state;

end
