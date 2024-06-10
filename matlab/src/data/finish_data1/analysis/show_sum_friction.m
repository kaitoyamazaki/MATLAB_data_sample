% 前回計算した, 合摩擦モーメントを描画する

% 変数の初期化
clear;

% 検索フォルダの追加
addpath('.', '-end');
addpath('res', '-end');

% .matファイルの読み込み
load_data = load('res/sum_friction_moment.mat');
Friction_moment = load_data.sum_friction_moment;

figure;

hold on;
grid on;

x = Friction_moment(:, 1);
y = Friction_moment(:, 2);

ylim()

plot(x, y);

hold off;