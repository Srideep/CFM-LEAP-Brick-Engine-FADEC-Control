%% Extract all logged signals
n1_dem = logsout.getElement('N1_demand').Values;
n1_lim = logsout.getElement('N1_limited').Values;
n1_act = logsout.getElement('N1_actual').Values;
egt    = logsout.getElement('EGT').Values;
ps3    = logsout.getElement('Ps3').Values;
wf     = logsout.getElement('Wf').Values;
fn     = logsout.getElement('Fn').Values;
af     = logsout.getElement('accel_flag').Values;
ef     = logsout.getElement('egt_flag').Values;
of     = logsout.getElement('ovsd_flag').Values;

%% Plot — 8 panel EICAS style
figure('Name', 'Integrated Thermo + Limit Test', ...
    'Position', [50 50 1200 900]);

% N1
subplot(4,2,1);
plot(n1_dem.Time, n1_dem.Data, 'b--', 'LineWidth', 1.2); hold on;
plot(n1_lim.Time, n1_lim.Data, 'r-', 'LineWidth', 1.5);
plot(n1_act.Time, n1_act.Data, 'k:', 'LineWidth', 1);
yline(104, 'r-', 'Redline');
yline(19, 'k:', 'Idle');
ylabel('N1 (%)');
title('N1 spool speed');
legend('Demand', 'Limited', 'Actual', 'Location', 'best');
grid on;

% EGT
subplot(4,2,2);
plot(egt.Time, egt.Data, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5);
yline(1060, 'r-', 'EGT Redline');
yline(1030, 'r:', 'Margin');
ylabel('EGT (°C)');
title('Exhaust gas temperature');
grid on;

% Thrust
subplot(4,2,3);
plot(fn.Time, fn.Data, 'Color', [0.1 0.6 0.3], 'LineWidth', 1.5);
ylabel('Fn (kN)');
title('Net thrust');
grid on;

% Fuel flow
subplot(4,2,4);
plot(wf.Time, wf.Data, 'Color', [0.5 0.2 0.7], 'LineWidth', 1.5);
ylabel('Wf (kg/hr)');
title('Fuel flow');
grid on;

% Ps3
subplot(4,2,5);
plot(ps3.Time, ps3.Data, 'Color', [0.3 0.6 0.6], 'LineWidth', 1.5);
ylabel('Ps3 (psia)');
title('Compressor discharge pressure');
grid on;

% Limit flags
subplot(4,2,6);
area(af.Time, af.Data * 3, 'FaceColor', [1 0.8 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6); hold on;
area(ef.Time, ef.Data * 2, 'FaceColor', [1 0.4 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
area(of.Time, of.Data * 1, 'FaceColor', [1 0 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
ylabel('Protection');
title('Limit flags');
yticks([0 1 2 3]); yticklabels({'Off', 'Overspd', 'EGT', 'Accel'});
legend('Accel', 'EGT', 'Overspeed', 'Location', 'best');
grid on;

% N1 demand vs limited (overlay for easy comparison)
subplot(4,2,7);
stairs(n1_dem.Time, n1_dem.Data, 'b-', 'LineWidth', 1.5); hold on;
plot(n1_lim.Time, n1_lim.Data, 'r-', 'LineWidth', 1.5);
ylabel('N1 (%)');
title('Demand vs limited (overlay)');
legend('Demand (instant)', 'Limited (rate clamped)');
grid on;

% EGT vs N1 (parametric — verifies thermo model shape)
subplot(4,2,8);
plot(n1_act.Data, egt.Data, '.', 'Color', [0.85 0.33 0.1], 'MarkerSize', 2);
xlabel('N1 (%)'); ylabel('EGT (°C)');
title('EGT vs N1 (should be parabolic)');
grid on;

sgtitle('Integrated test: thermo model + limit logic with feedback');

%% Save logged data to file
save('fadec_integrated_test_results.mat', 'logsout');
fprintf('\nResults saved to fadec_integrated_test_results.mat\n');
fprintf('Reload anytime with: load(''fadec_integrated_test_results.mat'')\n');