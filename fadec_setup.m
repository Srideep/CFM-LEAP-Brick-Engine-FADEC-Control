%% FADEC_SETUP.m
%  Run this script before opening the Simulink model.
%  Sets up workspace variables for the FADEC control system.
%
%  Architecture:
%    Simulink (this PC) <-- UDP --> fadec_bridge.py (Raspberry Pi + Build HAT)
%
%  Simulink reads sensor data, runs the FADEC control law, and sends
%  motor commands back to the Pi at ~50 Hz.

%% ========================================================================
%  NETWORK CONFIGURATION
%  ========================================================================
PI_IP   = '192.168.1.50';   % << CHANGE THIS to your Pi's IP address
PI_PORT = 5000;              % must match fadec_bridge.py --port
PC_PORT = 5001;              % must match fadec_bridge.py --simulink-port

%% ========================================================================
%  ENGINE PARAMETERS (loosely based on CFM LEAP-1B public data)
%  ========================================================================

% N1 operating range (mapped to motor speed 0-100%)
engine.N1_idle       = 19.0;     % N1 at idle (%)
engine.N1_max        = 100.0;    % N1 at TOGA (%)
engine.N1_redline    = 104.0;    % overspeed limit (%)

% EGT limits
engine.EGT_redline   = 1060;     % degrees C — max continuous
engine.EGT_start_lim = 725;      % degrees C — start limit

% Thrust (simplified)
engine.thrust_max_kN = 130;      % max rated thrust (LEAP-1B)

% Acceleration / deceleration limits
engine.dN1_max_accel = 8.0;      % max %N1 per second (spool-up rate)
engine.dN1_max_decel = 12.0;     % max %N1 per second (spool-down rate)

%% ========================================================================
%  THROTTLE LEVER ANGLE (TLA) to N1 DEMAND SCHEDULE
%  ========================================================================
%  Motor C encoder gives degrees. Map to N1 demand %.
%  Adjust these breakpoints after you measure your lever's actual range.

tla_schedule.lever_deg = [-30, 0, 30, 60, 90, 120];   % lever angle degrees
tla_schedule.n1_demand = [ 19, 19, 40, 65, 88, 100];   % N1 %

% In Simulink, use a 1-D Lookup Table block with these vectors.

%% ========================================================================
%  PID CONTROLLER GAINS
%  ========================================================================
%  Start with these, then tune live in Simulink external mode.

pid.Kp = 3.5;     % proportional gain
pid.Ki = 1.5;     % integral gain
pid.Kd = 0.1;     % derivative gain
pid.Ts = 0.02;    % sample time (50 Hz)
pid.output_min = 0;     % motor speed lower limit
pid.output_max = 100;   % motor speed upper limit

%% ========================================================================
%  THERMODYNAMIC MODEL COEFFICIENTS
%  ========================================================================
%  Simplified polynomial fits: f(N1) for each parameter.
%  These give plausible gauge readings, not real engine data.
%
%  EGT(N1)   = a2*N1^2 + a1*N1 + a0
%  Ps3(N1)   = b2*N1^2 + b1*N1 + b0      (compressor discharge pressure, psi)
%  Wf(N1)    = c1*N1 + c0                 (fuel flow, kg/hr)
%  Fn(N1)    = d2*N1^2                    (net thrust, kN)

thermo.egt_a2 = 0.120;     % EGT rises steeply at high N1
thermo.egt_a1 = 1.5;
thermo.egt_a0 = 350;       % EGT at idle ~= 350 + 1.5*19 + 0.085*19^2 ≈ 410°C

thermo.ps3_b2 = 0.004;
thermo.ps3_b1 = 0.1;
thermo.ps3_b0 = 14.7;      % atmospheric at idle

thermo.wf_c1  = 45;        % fuel flow scales linearly-ish
thermo.wf_c0  = -500;      % offset so idle Wf is low

thermo.fn_d2  = 0.013;     % thrust ~ N1 squared

%% ========================================================================
%  REVERSER LOGIC
%  ========================================================================

reverser.deploy_position  = 90;    % motor B degrees to open reverser cowl
reverser.stow_position    = 0;     % motor B degrees to close
reverser.idle_n1_reverse  = 25;    % N1% in reverse idle

%% ========================================================================
%  UDP PACKET FORMAT (must match fadec_bridge.py)
%  ========================================================================
%  Command packet (PC -> Pi):  12 bytes, little-endian
%    uint8   packet_type  (0x01)
%    uint8   reserved
%    single  motor_a_speed  (-100 to +100)
%    single  motor_b_pos    (degrees)
%    uint16  flags          (bit 0 = reverser deploy)
%
%  Sensor packet (Pi -> PC):   24 bytes, little-endian
%    uint8   packet_type  (0x02)
%    uint8   status
%    single  n1_speed     (-100 to +100)
%    single  n1_position  (cumulative degrees)
%    single  tla_degrees  (throttle lever angle)
%    single  reverser_pos (degrees)
%    uint32  timestamp_ms
%    uint16  reserved

disp('=== FADEC workspace loaded ===');
disp(['Pi address:  ', PI_IP, ':', num2str(PI_PORT)]);
disp(['PC listen:   port ', num2str(PC_PORT)]);
disp(['PID gains:   Kp=', num2str(pid.Kp), ...
      ' Ki=', num2str(pid.Ki), ' Kd=', num2str(pid.Kd)]);
disp(['Engine:      N1 idle=', num2str(engine.N1_idle), ...
      '% max=', num2str(engine.N1_max), '%']);
disp(' ');
disp('Next: open fadec_model.slx and run in External mode.');