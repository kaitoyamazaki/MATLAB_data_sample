% 部分拘束typeAのwrench coneを描画する
% 法線ベクトルf1を変化させた際のwrenchベクトルを表示する

clear;

addpath('.', '-end');
addpath('function', '-end');

% 法線ベクトルの定義
f1 = [0; 1; 0];
f2 = [1; 0; 0];
f3 = [-1; 0; 0];

% 法線ベクトルの位置ベクトルの定義
l1 = [-0.0075; 0.03; 0.0];
l2 = [-0.005; 0.0275; 0.0];
l3 = [0.005; -0.01; 0.0];

% レンチベクトルの集合をそれぞれ定義
m1 = [];
m2 = [];
m3 = [];

% エッジの計算
for i = 0:1
    % 力ベクトルの大きさを変更
    edit_f1 = i * f1;
    edit_f2 = i * f2;
    edit_f3 = i * f3;

    % 変更した力ベクトルのときのモーメントを計算
    now_moment1 = cross(l1, edit_f1);
    now_moment2 = cross(l2, edit_f2);
    now_moment3 = cross(l3, edit_f3);

    % 現在のレンチベクトルを定義
    wrench_f1 = [edit_f1(1) edit_f1(2) now_moment1(3)];
    wrench_f2 = [edit_f2(1) edit_f2(2) now_moment2(3)];
    wrench_f3 = [edit_f3(1) edit_f3(2) now_moment3(3)];

    % 現在のレンチベクトルを集合に格納
    m1 = [m1; wrench_f1];
    m2 = [m2; wrench_f2];
    m3 = [m3; wrench_f3];
end

% エッジを描画するために終点のみ取得
moment_point1 = m1(end, :);
moment_point2 = m2(end, :);
moment_point3 = m3(end, :);

% エッジを利用して
[points_x1, points_y1, points_z1] = make_plane(moment_point1, moment_point2);
[points_x2, points_y2, points_z2] = make_plane(moment_point2, moment_point3);
[points_x3, points_y3, points_z3] = make_plane(moment_point3, moment_point1);

% m1, m2, m3で代用できそう
[x1, y1, z1] = get_wrench_vec(m1);
[x2, y2, z2] = get_wrench_vec(m2);
[x3, y3, z3] = get_wrench_vec(m3);

% グラフ描画開始
figure;

hold on;

% wrenchに関する直線を描画
plot3(x1, y1, z1, 'Color', 'k', 'LineWidth', 1.5);
plot3(x2, y2, z2, 'Color', 'k', 'LineWidth', 1.5);
plot3(x3, y3, z3, 'Color', 'k', 'LineWidth', 1.5);

% 平面の作成

mesh1 = mesh(points_x1, points_y1, points_z1, 'FaceColor', 'b', 'LineStyle', 'none', 'FaceAlpha', 0.25);
mesh2 = mesh(points_x2, points_y2, points_z2, 'FaceColor', 'b', 'LineStyle', 'none', 'FaceAlpha', 0.25);
mesh3 = mesh(points_x3, points_y3, points_z3, 'FaceColor', 'b', 'LineStyle', 'none', 'FaceAlpha', 0.25);

plot_edge(moment_point1, moment_point2, moment_point3);

grid on;

% グラフタイトル
title('typeA in Wrench Space');

%ラベルの描画
xlabel('fx  [N]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', 'r');
ylabel('fy  [N]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', 'g');
zlabel('ω  [Nm]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', 'b');

%グラフの範囲を設定
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.03, 0.03]);

%view(128, 21.6);
%view(200, 55);
%view(160, 30);
%view(220, -45);
view(45, 50);

% 軸線を描画
line([min(xlim) max(xlim)], [0 0], [0 0], 'Color', 'red', 'LineWidth', 1.0);
line([0 0], [min(ylim), max(ylim)], [0 0], 'Color', 'green', 'LineWidth', 1.0);
line([0 0], [0 0], [min(zlim) max(zlim)], 'Color', 'blue', 'LineWidth', 1.0);

%for i = 0:0.01:1.0
    %[p1, box] = show_quiver(i, f1, l1);
    %pause(0.1);
    %if(i == 1)
        %disp('');
    %else
        %delete(p1);
        %delete(box);
    %end
%end

hold off;