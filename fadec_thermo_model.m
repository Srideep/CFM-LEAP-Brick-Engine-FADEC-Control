function [EGT, Ps3, Wf, Fn] = fadec_thermo_model(N1_pct)
%FADEC_THERMO_MODEL Simplified turbofan thermodynamic model.
%
%   Computes synthetic engine parameters from N1 spool speed.
%   These are polynomial curve fits giving plausible gauge readings
%   for the hobby FADEC — not real engine data.
%
%   Inputs:
%       N1_pct   - Fan spool speed (0-104%)
%
%   Outputs:
%       EGT      - Exhaust Gas Temperature (deg C)
%       Ps3      - Compressor discharge pressure (psia)
%       Wf       - Fuel flow (kg/hr)
%       Fn       - Net thrust (kN)
%
%   Usage in Simulink:
%       Drag a MATLAB Function block, paste this function.
%       Connect N1 feedback (scaled to %) to the input.
%       Route outputs to Dashboard Gauge blocks.

    % Clamp input
    N1 = max(0, min(104, N1_pct));

    % EGT: quadratic — rises steeply at high N1
    %   idle (~19%): ~410 C
    %   max  (100%): ~1200 C (above redline triggers limit logic)
    EGT = 0.085 * N1^2 + 1.5 * N1 + 350;

    % Compressor discharge pressure
    %   idle: ~16 psia
    %   max:  ~55 psia
    Ps3 = 0.004 * N1^2 + 0.1 * N1 + 14.7;

    % Fuel flow (kg/hr)
    %   idle: ~355 kg/hr
    %   max:  ~4000 kg/hr
    Wf = max(0, 45 * N1 - 500);

    % Net thrust (kN) — proportional to N1 squared
    %   idle: ~4.7 kN
    %   max:  ~130 kN
    Fn = 0.013 * N1^2;

end