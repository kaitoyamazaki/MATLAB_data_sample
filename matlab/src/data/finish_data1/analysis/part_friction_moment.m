% 指定した点で求めたモーメントを利用して合摩擦力モーメントを計算する

% 変数初期化
clear;

% 検索フォルダの追加
addpath('.', '-end');
addpath('res', '-end');

% .matファイルの読み込み
load_data = load('res/friction_moment.mat');
Friction_moment = load_data.friction_moment;

time = Friction_moment(:, 1);

moment1 = Friction_moment(:, 2);
moment2 = Friction_moment(:, 3);
moment3 = Friction_moment(:, 4);
moment4 = Friction_moment(:, 5);
moment5 = Friction_moment(:, 6);
moment6 = Friction_moment(:, 7);
moment7 = Friction_moment(:, 8);
moment8 = Friction_moment(:, 9);

x = time;

figure;

% 1行目のサブプロット
subplot(2, 2, 1);
y1 = moment1;
plot(x, y1);
title('重心1によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

subplot(2, 2, 2);
y2 = moment2;
plot(x, y2);
title('重心2によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

% 2行目のサブプロット
subplot(2, 2, 3);
y3 = moment3;
plot(x, y3);
title('重心3によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

subplot(2, 2, 4);
y4 = moment4;
plot(x, y4);
title('重心4によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

% Figure2の作成
figure;

% 1行目のサブプロット
subplot(2, 2, 1);
y5 = moment5;
plot(x, y5);
title('重心5によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

subplot(2, 2, 2);
y6 = moment6;
plot(x, y6);
title('重心6によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

% 2行目のサブプロット
subplot(2, 2, 3);
y7 = moment7;
plot(x, y7);
title('重心7によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;

subplot(2, 2, 4);
y8 = moment8;
plot(x, y8);
title('重心8によるモーメント');
xlabel('time [s]');
ylabel('\omega [Nm]');
ylim([-0.00025, 0.00025]);
hold on;
grid on;
