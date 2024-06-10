% 摩擦力を計算する

% 初期化する
clear;

% 検索フォルダの読み込み

addpath('.', '-end');
addpath('..', '-end');
addpath('res', '-end');

% ファイルの読み込み
Friction_direction = readmatrix('../partially_friction_points.csv');
position_vector = [0.0, -0.0125, 0.0350, 0.0, 0.035, 0.0, 0.015, -0.0125, -0.0350, 0.0, -0.035, 0.0125, 0.0350, 0.0, 0.015, 0.0, -0.015];
mass = [0.00159, 0.001069, 0.00159, 0.00159, 0.001069, 0.00159, 0.00318, 0.00318];
mu = 0.80;

Friction = mass .* 0.80;

rows = size(Friction_direction, 1);
min_rows = size(mass, 2);
cols = size(Friction_direction, 2);

nece_data = [];

for i = 2:rows
    data = [];
    data(end+1) = Friction_direction(i, 1);
    for j = 2:2:cols
        now_x = j;
        now_y = now_x+1;
        mass_num = now_x / 2;

        % 摩擦力を求めること
        % 方向を求める

        % 計算するための方向ベクトルと摩擦力を抽出

        % 方向ベクトル
        direction = [Friction_direction(i, now_x), Friction_direction(i, now_y)];

        % 摩擦力
        now_friction = Friction(mass_num);

        % 方向ベクトルのノルム入手
        now_norm = sqrt(sum(direction.^2));

        % 方向ベクトルの単位ベクトル
        unit_direction = [direction(1) / now_norm, direction(2) / now_norm];

        % 単位ベクトルを利用し, 摩擦力ベクトルを計算する
        friction_vector = now_friction * unit_direction;
        neg_friction_vector = -1 * friction_vector;
        %fprintf('now_x : %f\n', now_x);
        %fprintf('now_y : %f\n', now_y);

        now_force = [neg_friction_vector(1); neg_friction_vector(2); 0];
        now_pos = [position_vector(now_x); position_vector(now_y); 0];
        test_cross = cross(now_pos, now_force);
        
        Ans = test_cross(3);

        %disp(friction_vector);
        %disp(neg_friction_vector);
        %disp(Ans);
        data(end+1) = Ans;
    end
    disp(data);
    nece_data = [nece_data; data];
end

friction_moment = nece_data;
save('res/friction_moment.mat', 'friction_moment');