% シミュレータで得たデータを素に, レンチ空間をスパンする

% 初期化
clear;

% 検索フォルダの読み込み
addpath('.', '-end');
addpath('function', '-end');
addpath('data', '-end');
addpath('figure', '-end');

% フラフデータの取得と出力
openfig('figure/wrench_typeA_20.fig');

hold on;

view(200, 20);

% 法線データの定義
f1 = 1;
f2 = 1;
f3 = 1;


% 力データの読み込み
force_r = readmatrix('data/force_r_typeA.csv');
force_l = readmatrix('data/force_l_typeA.csv');

% 力データの編集
force_c1 = force_r(:, 2);
force_c2 = force_r(:, 1);
force_c3 = force_l(:, 2);

% 法線データの位置ベクトルを定義
l1 = [0.0075; 0.03; 0.0];
l2 = [0.005; 0.0275; 0.0];
l3 = [-0.0125; 0.03; 0.0];

all_wrench = [];

trial_num = size(force_c1, 1);

for i = 1:trial_num

    f1 = force_c1(i, 1);
    f2 = force_c2(i, 1);
    f3 = force_c3(i, 1);

    f1 = -1 * f1;
    %f2 = -1 * f2;
    f3 = -1 * f3;

    wrench = calc_now_wrench(l1, f1, l2, f2, l3, f3);

    all_wrench = [all_wrench; wrench];

    p = plot3([0, wrench(1)], [0, wrench(2)], [0, wrench(3)], 'Color', [1.0, 0.33, 0.65], 'LineWidth', 4.0);

    pause(0.05);

    delete(p);
    %wrench_str = sprintf('[%.5f %.5f %.5f]', wrench(1), wrench(2), wrench(3));
    %text = sprintf('%d番目のwrenchベクトル : %s', i, wrench_str);
    %disp(text);
end

hold off;