function qdd_filt = acc_filter(n, ws, wc, input, mode, path_prefix)
%% acc_filtering.m
% @brief: offline zero-phase butterworth filtering for qdd (joint acceleration)
% @params[in] n: order of filter (5 by default)
% @params[in] ws: sampling frequency (10 by default)
% @params[in] wc: cut-off frequency (3 by default)
% @params[in] input: qdd_raw for sensor reading, qd_filt for derivative
% @params[in] mode: "sensor" or "derivate"
% @params[out] qdd_filtered: filtered joint acceleration data

% cut-off frequency
wn = wc / (ws / 2);		
% low-pass filter :n是滤波器的阶数，根据需要选择合适的整数，Wn是归一化截止频率，又叫自然频率，Wn = 截止频率*2/采样频率，如果要留下小于截至频率的信号，
[b, a] = butter(n, wn, 'low');
% sampling period  
Ts = 1 / ws;    

%% FILTERING
if mode == "derivate"
	len = length(input(:, 1));	 % input is qd_filt and length is 200
	qdd_pre_filt = zeros(len, 6);

	for k = 2:len-1
		qdd_pre_filt(k, :)= (input(k+1,:) - input(k-1,:)) / (2 * Ts);
	end
	qdd_pre_filt(1, :) = qdd_pre_filt(2, :);
	qdd_pre_filt(len, :) = qdd_pre_filt(len-1, :);

elseif mode == "sensor"
	qdd_pre_filt = input;  % input is qd_raw
end
qdd_filt = filtfilt(b, a, qdd_pre_filt);
% 中值滤波和移动平均滤波
for i = 1:6
    qdd_med_filt = medfilt1(qdd_filt(:, i), 10);
    qdd_smooth_filt = smooth(qdd_med_filt, 20, 'rloess');
    qdd_filt(:, i) = qdd_smooth_filt;
end
% qdd_filt = smooth(qdd_pre_filt, 'rloess');

%% VISUALIZATION
for i = 1:6
	figure(i + 12); 
	plot(qdd_pre_filt(:,i), 'g', 'LineWidth', 1.0); hold on;
	plot(qdd_filt(:, i), 'r', 'LineWidth', 0.5); hold off;
	title(['第', num2str(i), '关节加速度滤波结果'], 'FontSize', 17, 'FontName', '宋体');
	ylabel('关节加速度(rad/s^{2})', 'FontSize', 17, 'FontName', '宋体');
	legend('滤波前', '滤波后', 'FontName', '宋体', 'FontSize', 12);
    print(i + 12, '-dpng', '-r600', [path_prefix, 'Joint', num2str(i), 'Acc.png'])
end

end
