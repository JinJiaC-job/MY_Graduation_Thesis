function t_filt = trq_filter(n, ws, wc, t_raw, path_prefix)
%% torque_filter.m
% @brief: offline zero-phase butterworth filtering for torque
% @params[in] n: order of filter (5 by default)
% @params[in] ws: sampling frequency (10 by default)
% @params[in] wc: cut-off frequency (3 by default)
% @params[in] t_raw: raw joint torque data from sensors
% @params[out] t_filt: filtered joint torque data

% cut-off frequency
wn = wc / (ws / 2);		
% low-pass filter
[b, a] = butter(n, wn, 'low');
% zero-phase digital filtering
t_filt = filtfilt(b, a, t_raw);
% t_filt = smooth(t_raw, 6, 'rloess');
% t_smooth_filt = smooth(t_filt, 6, 'rloess');
% 中值滤波和移动平均滤波
% for i = 1:6
%     t_med_filt = medfilt1(t_filt(:, i), 10);
%     t_smooth_filt = smooth(t_med_filt, 20, 'rloess');
%     t_filt(:, i) = t_smooth_filt;
% end
%% VISUALIZATION
for i = 1:6
    if i==1
	figure(i + 18); 
	plot(t_raw(:, i), 'g', 'LineWidth', 1.0); hold on;
%     t_smooth_filt = smooth(t_filt(:, i), 6, 'rloess');

	plot(t_filt(:, i), 'r', 'LineWidth', 0.5); hold off;
	title(['第', num2str(i), '关节输出力滤波结果'], 'FontSize', 17, 'FontName', '宋体');
    xlabel('采样点', 'FontSize', 17, 'FontName', '宋体');
	ylabel('关节力矩(N)', 'FontSize', 17, 'FontName', '宋体');
	legend('滤波前', '滤波后', 'FontSize', 12, 'FontName', '宋体');
    print(i + 18, '-dpng', '-r600', [path_prefix, 'Joint', num2str(i), 'Torque.png']);
    end

    figure(i + 18); 
	plot(t_raw(:, i), 'g', 'LineWidth', 1.0); hold on;
%     t_smooth_filt = smooth(t_filt(:, i), 6, 'rloess');

	plot(t_filt(:, i), 'r', 'LineWidth', 0.5); hold off;
	title(['第', num2str(i), '关节输出力矩滤波结果'], 'FontSize', 17, 'FontName', '宋体');
    xlabel('采样点', 'FontSize', 17, 'FontName', '宋体');
	ylabel('关节力矩(Nm)', 'FontSize', 17, 'FontName', '宋体');
	legend('滤波前', '滤波后', 'FontSize', 12, 'FontName', '宋体');
    print(i + 18, '-dpng', '-r600', [path_prefix, 'Joint', num2str(i), 'Torque.png']);
end

end



