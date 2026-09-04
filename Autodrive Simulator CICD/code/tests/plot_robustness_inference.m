function plot_robustness_inference(sim, test_results, thresholds)
    figure('Name', 'AV Robustness Inference Dashboard', 'Color', 'w', 'Position', [100 100 1200 800]);
    
    % --- 1. Safety Margin Normalized Bar Chart ---
    subplot(2,2,1);
    metrics = test_results(:,1);
    values = cell2mat(test_results(:,2));
    limits = cell2mat(test_results(:,3));
    utilization = (values ./ limits) * 100; % Percentage of threshold used
    
    b = barh(utilization, 'FaceColor', 'flat');
    for k = 1:size(utilization,1)
        if utilization(k) > 100, b.CData(k,:) = [0.8 0 0]; % Red for Fail
        else, b.CData(k,:) = [0 0.6 0.3]; end % Green for Pass
    end
    set(gca, 'YTickLabel', metrics, 'XLim', [0 120]);
    xline(100, '--r', 'Threshold Limit', 'LabelHorizontalAlignment', 'right');
    xlabel('Threshold Utilization (%)');
    title('Safety Margin (Lower is More Robust)');
    grid on;

    % --- 2. Error Envelopes (Lateral & Longitudinal) ---
    subplot(2,2,2);
    hold on;
    % Shaded "Safe Zone"
    fill([sim.t(1) sim.t(end) sim.t(end) sim.t(1)], ...
         [-thresholds.max_xterr -thresholds.max_xterr thresholds.max_xterr thresholds.max_xterr], ...
         [0.9 1 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(sim.t, sim.xterr, 'LineWidth', 1.5, 'Color', [0 0.4 0.8], 'DisplayName', 'Cross-Track Error');
    ylabel('Error (m)');
    xlabel('Time (sec)');
    title('Spatial Robustness: Error vs. Safe Envelope');
    legend('Location', 'best');
    grid on;

    % --- 3. State Transition & Reaction Latency (TTA) ---
    subplot(2,2,3);
    yyaxis left;
    stairs(sim.t, sim.VC_state, 'LineWidth', 2, 'DisplayName', 'VC State');
    ylabel('State ID');
    ylim([0 5]);
    
    yyaxis right;
    if isfield(sim, 'wpt_status_change')
        plot(sim.t, sim.wpt_status_change, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Jamming Trigger');
        ylabel('Fault Status');
    end
    title('Temporal Robustness: State Reaction to Faults');
    legend('Location', 'best');
    grid on;

    % --- 4. Dynamic Stability Phase Plot (Jerk vs Accel) ---
    subplot(2,2,4);
    plot(sim.alat, sim.jlat, 'LineWidth', 1, 'Color', [0.5 0.2 0.6]);
    hold on;
    % Draw "Comfort/Stability Box"
    rectangle('Position', [-thresholds.max_alat, -thresholds.max_jerk, ...
              thresholds.max_alat*2, thresholds.max_jerk*2], ...
              'EdgeColor', 'r', 'LineStyle', '--');
    xlabel('Lateral Accel (m/s^2)');
    ylabel('Lateral Jerk (m/s^3)');
    title('Control Robustness: Stability Phase Space');
    grid on;
    
    sgtitle(['Robustness Inference: ', datestr(now)]);
end