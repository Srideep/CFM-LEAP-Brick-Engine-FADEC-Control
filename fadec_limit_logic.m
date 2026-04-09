function [N1_limited, limited_flag, egt_flag, overspeed_flag] = ...
    fadec_limit_logic(N1_demand, N1_actual, EGT, Ts)
%FADEC_LIMIT_LOGIC  FADEC limit protection logic.
%
%   Implements three protection layers, just like a real FADEC:
%     1. Acceleration/deceleration rate limiter (prevents surge)
%     2. EGT redline protection (reduces N1 if EGT exceeds limit)
%     3. Overspeed protection (hard N1 cap)
%
%   Inputs:
%       N1_demand  - Desired N1 from PID controller (%)
%       N1_actual  - Current N1 from encoder feedback (%)
%       EGT        - Current exhaust gas temperature (deg C)
%       Ts         - Sample time (seconds)
%
%   Outputs:
%       N1_limited     - Rate-limited and protected N1 command (%)
%       limited_flag   - 1 if accel/decel limiter is active
%       egt_flag       - 1 if EGT protection is active
%       overspeed_flag - 1 if overspeed protection is active

    % --- Constants ---
    persistent N1_prev;
    if isempty(N1_prev)
        N1_prev = 19.0;  % start at idle
    end

    MAX_ACCEL  = 8.0;     % max %N1/sec acceleration
    MAX_DECEL  = 12.0;    % max %N1/sec deceleration
    EGT_RED    = 1060;    % EGT redline (deg C)
    EGT_MARGIN = 30;      % start reducing before redline
    N1_RED     = 104.0;   % N1 overspeed limit (%)

    % --- Acceleration / deceleration limiter ---
    delta = N1_demand - N1_prev;
    max_up   =  MAX_ACCEL * Ts;
    max_down = -MAX_DECEL * Ts;
    delta_clamped = max(max_down, min(max_up, delta));
    N1_rate_limited = N1_prev + delta_clamped;
    limited_flag = (abs(delta) > abs(delta_clamped) + 0.01);

    % --- EGT protection ---
    egt_flag = 0;
    N1_egt_limited = N1_rate_limited;
    if EGT > (EGT_RED - EGT_MARGIN)
        % Proportional reduction as EGT approaches redline
        egt_excess = EGT - (EGT_RED - EGT_MARGIN);
        reduction = egt_excess * 0.5;  % 0.5% N1 per deg C over margin
        N1_egt_limited = N1_rate_limited - reduction;
        N1_egt_limited = max(19.0, N1_egt_limited);  % never below idle
        egt_flag = 1;
    end

    % --- Overspeed protection ---
    overspeed_flag = 0;
    N1_final = N1_egt_limited;
    if N1_final > N1_RED
        N1_final = N1_RED;
        overspeed_flag = 1;
    end

    % Clamp to idle floor
    N1_final = max(19.0, N1_final);

    % Update persistent state
    N1_prev = N1_final;

    % Output
    N1_limited = N1_final;

end