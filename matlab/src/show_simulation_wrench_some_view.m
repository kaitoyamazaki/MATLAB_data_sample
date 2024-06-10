% シミュレータから取得した力ベクトルを用いて, レンチ空間を作成する

% 初期化する
clear;

% 検索フォルダの読み込み

% 検索フォルダの読み込み
addpath('.', '-end');
addpath('function', '-end');
addpath('data', '-end');
addpath('figure', '-end');
addpath('data/finish_data1', '-end');
addpath('data/finish_data2', '-end');
addpath('data/finish_data3', '-end');
addpath('data/finish_data4', '-end');
addpath('data/finish_data5', '-end');

% 秒数の設定
simulation_seconds = 9.11;

% .figファイルを読み込む
% .figファイルはtypeAのレンチ空間
fig = openfig('figure/wrench_typeA_20.fig', 'reuse'); % your_figure.fig を読み込みます

% サブプロットの作成
figure;
subplot(2, 2, 1);
h1 = copyobj(allchild(get(fig, 'CurrentAxes')), gca); % 既存のプロットをコピー
title('all view');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(160, 20); % XY平面からの視点
hold on;
grid on;

subplot(2, 2, 2);
h2 = copyobj(allchild(get(fig, 'CurrentAxes')), gca); % 既存のプロットをコピー
title('XZ Plane View');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(180, 0); % XZ平面からの視点
hold on;
grid on;

subplot(2, 2, 3);
h3 = copyobj(allchild(get(fig, 'CurrentAxes')), gca); % 既存のプロットをコピー
title('YZ Plane View');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(90, 0); % YZ平面からの視点
hold on;
grid on;

subplot(2, 2, 4);
h4 = copyobj(allchild(get(fig, 'CurrentAxes')), gca); % 既存のプロットをコピー
title('XY Plane View');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(180, 90); % 任意の視点
hold on;
grid on;

sgtitle('typeA in Wrench Space from different views $(d = 20[mm])$', 'interpreter', 'latex');

% 法線データの定義
f1 = 1;
f2 = 1;
f3 = 1;

% 力データの読み込み
force_r = readmatrix('data/finish_data1/force_r_typeA.csv');
force_l = readmatrix('data/finish_data1/force_l_typeA.csv');

% 力データの編集
force_c1 = force_r(:, 3);
force_c2 = force_r(:, 2);
force_c3 = force_l(:, 3);

% 法線データの位置ベクトルを定義
l1 = [0.0075; 0.03; 0.0];
l2 = [0.005; 0.0275; 0.0];
l3 = [-0.0125; 0.03; 0.0];

all_wrench = [];

trial_num = size(force_c1, 1);
pause_time = simulation_seconds / trial_num;
pause_time = 0.00125;

pause(15);

for i = 1:trial_num
    f1 = force_c1(i, 1);
    f2 = force_c2(i, 1);
    f3 = force_c3(i, 1);

    f1 = -1 * f1;
    %f2 = -1 * f2;
    f3 = -1 * f3;

    wrench = calc_now_wrench(l1, f1, l2, f2, l3, f3);

    all_wrench = [all_wrench; wrench];

    x = [0, wrench(1)];
    y = [0, wrench(2)];
    z = [0, wrench(3)];


    % 各サブプロットに新しいデータを追加
    subplot(2, 2, 1);
    p1 = plot3(x, y, z, 'Color', [1.0, 0.33, 0.65], 'LineWidth', 4.0); % 新しい直線を追加
    
    subplot(2, 2, 2);
    p2 = plot3(x, y, z, 'Color', [1.0, 0.33, 0.65], 'LineWidth', 4.0); % 新しい直線を追加
    
    subplot(2, 2, 3);
    p3 = plot3(x, y, z, 'Color', [1.0, 0.33, 0.65], 'LineWidth', 4.0); % 新しい直線を追加
    
    subplot(2, 2, 4);
    p4 = plot3(x, y, z, 'Color', [1.0, 0.33, 0.65], 'LineWidth', 4.0); % 新しい直線を追加

    % 描画を更新
    drawnow;
    pause(pause_time);
    delete(p1);
    delete(p2);
    delete(p3);
    delete(p4);

end

%for i = 1:length(t)
    %x = t(1:i);
    %y = sin(t(1:i)); % 例としてsin関数で変化させる
    %z = cos(t(1:i)); % 例としてcos関数で変化させる
    
    %% 各サブプロットに新しいデータを追加
    %subplot(2, 2, 1);
    %plot3(x, y, z, 'k'); % 新しい直線を追加
    
    %subplot(2, 2, 2);
    %plot3(x, y, z, 'k'); % 新しい直線を追加
    
    %subplot(2, 2, 3);
    %plot3(x, y, z, 'k'); % 新しい直線を追加
    
    %subplot(2, 2, 4);
    %plot3(x, y, z, 'k'); % 新しい直線を追加
    
    %% 描画を更新
    %drawnow;
    %pause(0.05); % アニメーションの速度調整
%end
