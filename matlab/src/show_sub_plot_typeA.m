% subplotのテスト

clear;

% 部分拘束typeA
openfig('figure/wrench_typeA_20.fig');
%openfig('figure/wrench_typeA_25.fig');
%openfig('figure/wrench_typeA_30.fig');
%openfig('figure/wrench_typeA_35.fig');
%openfig('figure/wrench_typeA_40.fig');

ax = gca;

% 新しい図を作成
figure;

% サブプロット1
subplot(2, 2, 1);
copyobj(ax.Children, gca);
title('typeA in Wrench Space $(d = 20[mm])$', 'interpreter', 'latex');
%title('typeA in Wrench Space $(d = 25[mm])$', 'interpreter', 'latex');
%title('typeA in Wrench Space $(d = 30[mm])$', 'interpreter', 'latex');
%title('typeA in Wrench Space $(d = 35[mm])$', 'interpreter', 'latex');
%title('typeA in Wrench Space $(d = 40[mm])$', 'interpreter', 'latex');

xlabel('fx [N]', 'Color', 'r');
ylabel('fy [N]', 'Color', 'g');
zlabel('\omega [Nm]', 'Color', 'b');

view(45, 50);
grid on;

% サブプロット2

subplot(2, 2, 2);
copyobj(ax.Children, gca); % 現在のサブプロットにデータをコピー
title('fy - \omega plane');
xlabel('fx [N]', 'Color', 'r');
ylabel('fy [N]', 'Color', 'g');
zlabel('\omega [Nm]', 'Color', 'b');
view(90, 0); % 上からの視点
grid on;

% サブプロット 3: 別の視点
subplot(2, 2, 3); % 2×2のグリッドの位置3
copyobj(ax.Children, gca); % 現在のサブプロットにデータをコピー
title('fx - \omega plane');
xlabel('fx [N]', 'Color', 'r');
ylabel('fy [N]', 'Color', 'g');
zlabel('\omega [Nm]', 'Color', 'b');
view(0, 0.0); % 側面からの視点
grid on;

% サブプロット 4: カスタム視点
subplot(2, 2, 4); % 2×2のグリッドの位置4
copyobj(ax.Children, gca); % 現在のサブプロットにデータをコピー
title('typeA in Wrench Space from different view');
xlabel('fx [N]', 'Color', 'r');
ylabel('fy [N]', 'Color', 'g');
zlabel('\omega [Nm]', 'Color', 'b');
view(170.0, 20.0); % カスタム視点
grid on;