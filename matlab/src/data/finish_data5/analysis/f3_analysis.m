% f1のデータを解析するプログラムを記述

% 初期化
clear;

% データがある箇所のフォルダを検索フォルダに追加
addpath('..');

% 力データの読み込み

force_l = readmatrix('../force_l_typeA.csv');

% 力データの編集
time = force_l(:, 1);
force_c3 = force_l(:, 3);
force_c3 = -1 .* force_c3;

force_pos = force_c3(force_c3 >= 0);
force_neg = force_c3(force_c3 < 0);

time_pos = time(force_c3 >= 0);
time_neg = time(force_c3 < 0);

% 値を導出

% 平均値
mean_value_f1 = mean(force_c3);

% 中央値
median_value_f1 = median(force_c3);

% 分散
variance_value_f1 = var(force_c3);

% 標準値
std_deviation_f1 = std(force_c3);

% 移動平均
window_size = 3;
moving_ave_f1 = movmean(force_c3, window_size);

figure;
grid on;
hold on;

title('Magnitude of force at contact point3');
xlabel('time [s]');
ylabel('f [N]');

ylim([-1, 1]);

scatter(time_pos, force_pos, 'filled', 'b');
scatter(time_neg, force_neg, 'filled', 'r');