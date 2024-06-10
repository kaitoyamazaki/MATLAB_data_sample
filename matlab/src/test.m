% .figファイルを読み込む
fig = openfig('figure/wrench_typeA_20.fig', 'reuse'); % your_figure.fig を読み込みます

% サブプロットの作成
figure;
subplot(2, 2, 1);
h1 = copyobj(allchild(get(fig, 'CurrentAxes')), gca); % 既存のプロットをコピー
title('XY Plane View');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(0, 90); % XY平面からの視点
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
view(0, 0); % XZ平面からの視点
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
title('Isometric View');
xlabel('fx [N]');
ylabel('fy [N]');
zlabel('w [Nm]');
%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);
view(3); % 任意の視点
hold on;
grid on;

sgtitle('3D Line from Different Views');

% 動的に直線を追加する
t = linspace(0, 10, 100); % 例としてデータを生成
for i = 1:length(t)
    x = t(1:i);
    y = sin(t(1:i)); % 例としてsin関数で変化させる
    z = cos(t(1:i)); % 例としてcos関数で変化させる
    
    % 各サブプロットに新しいデータを追加
    subplot(2, 2, 1);
    plot3(x, y, z, 'k'); % 新しい直線を追加
    
    subplot(2, 2, 2);
    plot3(x, y, z, 'k'); % 新しい直線を追加
    
    subplot(2, 2, 3);
    plot3(x, y, z, 'k'); % 新しい直線を追加
    
    subplot(2, 2, 4);
    plot3(x, y, z, 'k'); % 新しい直線を追加
    
    % 描画を更新
    drawnow;
    pause(0.05); % アニメーションの速度調整
end
