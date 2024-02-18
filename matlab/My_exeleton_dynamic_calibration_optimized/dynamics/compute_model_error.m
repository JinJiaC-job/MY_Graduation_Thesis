%% compute_model_error.m

% 数据采样点
sample_point = 400;%采样点数
n = linspace(1, sample_point, sample_point);

% 巴特沃斯滤波器（一阶滤波器）
sample_p = 0.001;
cutoff_f = 100;
a = 1 / (1 + 1 / (2 * pi * sample_p * cutoff_f));
b0 = 1 / (1 + (1 / (pi * sample_p * cutoff_f)));
b1 = b0;
a0 = -(1 - (1 / (pi * cutoff_f * sample_p))) / (1+(1/(pi * cutoff_f * sample_p)));

% 计算辨识关节扭矩
T_idy = zeros(400, 6);
T_idy_filt = zeros(400, 6);
for kk = 1:400
    disp(['FIGURING OUT No.', num2str(kk), ' point!']);
    % computation method
    % T_idy(kk, :) = NewtonEuler_verify_math(q_ds(kk, :), qd_ds(kk, :), qdd_ds(kk, :));
    T_idy(kk, :) = NewtonEuler_verify_math(q_ds(kk, :), qd_ds(kk, :), qdd_ds(kk, :), "P_link") * 1e-3;
    %% filtering
    if (kk > 1)
        T_idy_filt(kk, :) = b0 * T_idy(kk, :) + b1 * T_idy(kk-1, :) + a0 * T_idy_filt(kk-1, :);
    else
        T_idy_filt(1, :) = T_idy(1, :);
    end
end

% figure(1);
error = zeros(sample_point, 6);
for ii = 1:6
    % figure(ii);
    subplot(2, 4, ii)
    % plot(n, t_ds(:, ii), 'b'); hold on;
    plot(n, t_filt(:, ii), 'b'); hold on;
    plot(n, T_idy(:, ii), 'r');
    plot(n, t_filt(:, ii) - T_idy(:, ii), 'g'); hold off;
    legend('采样力矩', '辨识力矩', '相对误差', 'FontName', '宋体', 'FontSize', 12);
    ylabel('力矩(Nm)', 'FontSize', 17, 'FontName', '宋体');
    title(['第', num2str(ii), '关节力矩前馈误差'], 'FontSize', 17, 'FontName', '宋体');
    print(ii, '-dpng', '-r600', ['.\figs\第', num2str(ii), '关节辨识参数验证'])
    error(:, ii) = abs(t_ds(:, ii) - T_idy(:, ii));
end
