function [ds_q, ds_qd, ds_qdd, ds_tau] = downsampling(raw_q, raw_qd, raw_tau, down_size)
% @brief: downsample raw data from robot sensors
% @param[in] raw_q, raw_qd, raw_qdd, raw_tau:
%            初始角度，角速度，角加速度，力矩数据
% @param[in] down_size: 采样点数多少
% @param[out] ds_q, ds_qd, ds_qdd, ds_tau:
%             采样角度，角速度，角加速度，力矩信息

% filtering
% uni_data = unique([raw_q, raw_qd, raw_qdd, raw_tau], 'rows');
% uni_q = uni_data(:, 1:6);
% uni_qd = uni_data(:, 7:12);
% uni_qdd = uni_data(:, 13:18);
% uni_tau = uni_data(:, 19:24);

ds_q = zeros(down_size, 6);
ds_qd = zeros(down_size, 6);
ds_qdd = zeros(down_size, 6);
ds_tau = zeros(down_size, 6);

% downsampling
step = fix(size(raw_q, 1)/down_size);%fix：截尾取整; size(raw_q, 1):返回raw_q的行数，采样点远少于实际运行采集的数据点
for i = 1:down_size
    ds_q(i, :) = raw_q(step * i, :);
    ds_qd(i, :) = raw_qd(step * i, :);
%     ds_qdd(i, :) = raw_qdd(step * i, :);
    ds_tau(i, :) = raw_tau(step * i, :);
end

len = length(ds_qd(:, 1));
for k = 2:len-1
	ds_qdd(k, :)= (ds_qd(k+1,:) - ds_qd(k-1,:)) / (2 * (20/down_size));%Ts
end
ds_qdd(1, :) = ds_qdd(2, :);
ds_qdd(len, :) = ds_qdd(len-1, :);


