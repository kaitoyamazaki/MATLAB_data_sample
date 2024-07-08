% 取得したデータを解析するプログラム　試作

% 初期化
clear;

addpath('.', '-end');
addpath('data', '-end');
addpath('data/new_data', '-end');
addpath('..', '-end');
addpath('use_data', '-end');

% 取得したdataは時間, ハンドの中心の位置座標, 物体の重心位置(xとy), 物体の代表点(重心1~8の位置座標xとy)
filepath = 'data/new_data/検証用のデータ.csv';
data = readmatrix(filepath);

% パラメータ設定
m = 0.01484;
mu = 0.2;
mass = [0.00159, 0.001069, 0.00159, 0.00159, 0.001069, 0.00159, 0.00318, 0.00318];

friction_obj = mu * m;
friction = mass .* mu;

% 最初に取得したデータの行数と列数
row = size(data, 1);
col = size(data, 2);
time_col = 1;
hand_ori_col = 1;
hand_pos_col = 2;
cog_col = 2;
representative_col = col - time_col - hand_ori_col - hand_pos_col - cog_col;

% 取得したデータを処理のために配列を細分化
% 時間データを抽出
time = data(:, time_col);

% ハンドの姿勢を抽出
hand_ori = data(:, time_col+hand_ori_col);

% ハンドの位置座標を抽出
s_col = time_col+hand_ori_col+1;
e_col = time_col+hand_ori_col+hand_pos_col;
hand_pos = data(:, s_col:e_col);

% 重心の位置座標を抽出
s_col = e_col+1;
e_col = e_col+cog_col;
cog_pos = data(:, s_col:e_col);

% 代表点の位置座標を抽出
s_col = e_col+1;
e_col = col;
representative_pos = data(:, s_col:e_col);

time_col = size(time,2);
hand_col = size(hand_pos,2);
cog_col = size(cog_pos,2);
representative_col = size(representative_pos,2);

% 時間当たりの偏差を格納する配列を用意
dp_time = zeros(row, time_col);
dp_hand_pos = zeros(row, hand_col);
dp_cog_pos = zeros(row, cog_col);
dp_representative_pos = zeros(row, representative_col);

% 時間当たりの偏差を導出
for i = 1:row
    if(i == 1)
        calc_time = time(i,:) - time(i,:);
        calc_hand_pos = hand_pos(i,:) - hand_pos(i,:);
        calc_cog_pos = cog_pos(i,:) - cog_pos(i,:);
        calc_representative_pos = representative_pos(i,:) - representative_pos(i,:);
    else
        calc_time = time(i,:) - time(i-1,:);
        calc_hand_pos = hand_pos(i,:) - hand_pos(i-1,:);
        calc_cog_pos = cog_pos(i,:) - cog_pos(i-1,:);
        calc_representative_pos = representative_pos(i,:) - representative_pos(i-1,:);
    end

    dp_time(i,:) = calc_time;
    dp_hand_pos(i,:) = calc_hand_pos;
    dp_cog_pos(i,:) = calc_cog_pos;
    dp_representative_pos(i,:) = calc_representative_pos;
end

% 時間当たりの速度を格納する変数を用意
velocity_hand = zeros(row, hand_col);
velocity_cog = zeros(row, cog_col);
velocity_representative_points = zeros(row, representative_col);

% 時間当たりの偏差データから速度データを計算
for i = 1:row
    t = dp_time(i,1);
    v_hand = dp_hand_pos(i,:) / t;
    v_cog = dp_cog_pos(i,:) / t;
    v_rep = dp_representative_pos(i,:) / t;

    velocity_hand(i, :) = v_hand;
    velocity_cog(i,:) = v_cog;
    velocity_representative_points(i,:) = v_rep;
end

% NaN要素を0に変換
velocity_hand(isnan(velocity_hand)) = 0;
velocity_cog(isnan(velocity_cog)) = 0;
velocity_representative_points(isnan(velocity_representative_points)) = 0;

% ハンド座標系をベースにするための座標変換を実施
trans_velocity_hand = zeros(size(velocity_hand));
trans_velocity_cog = zeros(size(velocity_cog));
trans_velocity_representative_points= zeros(size(velocity_representative_points));

for i = 1:row
    theta = -hand_ori(i,1);
    rotation_matrix = [cos(theta), -sin(theta);
                       sin(theta), cos(theta)];
    
    v_hand = rotation_matrix * velocity_hand(i,:).';
    v_cog = rotation_matrix * velocity_cog(i,:).';
    trans_v_rep = zeros(1,16);
    for j = 1:2:15
        v = [velocity_representative_points(i,j), velocity_representative_points(i,j+1)];
        v_rep = rotation_matrix * v.';
        v_rep = v_rep.';
        trans_v_rep(j) = v_rep(1);
        trans_v_rep(j+1) = v_rep(2);
    end

    trans_velocity_hand(i,:) = v_hand.';
    trans_velocity_cog(i,:) = v_cog.';
    trans_velocity_representative_points(i,:) = trans_v_rep;
end

%% 単位ベクトルを格納する配列を用意
%unit_velocity_hand = zeros(row, hand_col);
%unit_velocity_cog = zeros(row, cog_col);
%unit_velocity_representative_points = zeros(row, representative_col);

%% 時間当たりの速度から単位ベクトルを計算
%for i = 1:row
    %% 最初にハンドの対しての処理
    %norm_v_hand = sqrt(velocity_hand(i,1)^2 + velocity_hand(i,2)^2);
    %uv_hand = velocity_hand(i,:) / norm_v_hand;

    %% 次に重心に対しての処理
    %norm_v_cog = sqrt(velocity_cog(i,1)^2 + velocity_cog(i,2)^2);
    %uv_cog = velocity_cog(i,:) / norm_v_cog;

    %uv_data = zeros(1, 16);
    %for j = 1:2:15
        %v = [velocity_representative_points(i,j), velocity_representative_points(i,j+1)];
        %norm_v = sqrt(v(1)^2 + v(2)^2);
        %uv = v / norm_v; 
        %uv_data(j) = uv(1);
        %uv_data(j+1) = uv(2);
    %end

    %unit_velocity_hand(i, :) = uv_hand;
    %unit_velocity_cog(i, :) = uv_cog;
    %unit_velocity_representative_points(i, :) = uv_data;
%end

%unit_velocity_hand(isnan(unit_velocity_hand)) = 0;
%unit_velocity_cog(isnan(unit_velocity_cog)) = 0;
%unit_velocity_representative_points(isnan(unit_velocity_representative_points)) = 0;

%% 摩擦力の配列をそれぞれ用意
%friction_hand = zeros(row, hand_col);
%friction_cog = zeros(row, cog_col);
%friction_representative_points = zeros(row, representative_col);

%% 単位ベクトルの値から摩擦力を計算
%for i = 1:row
    %f_hand = -friction_obj * unit_velocity_hand(i, :);
    %f_cog = -friction_obj * unit_velocity_cog(i, :);

    %friction_data = zeros(1, 16);
    %for j = 1:2:15
        %v = [unit_velocity_representative_points(i,j), unit_velocity_representative_points(i, j+1)];
        %f_col = idivide(int32(j),2)+1;
        %f_rep = -friction(f_col) * v;
        %friction_data(j) = f_rep(1);
        %friction_data(j+1) = f_rep(2);
    %end

    %friction_hand(i, :) = f_hand;
    %friction_cog(i, :) = f_cog;
    %friction_representative_points(i, :) = friction_data;
%end

%% 摩擦力のデータからモーメントを計算. その結果を格納するための変数を用意
%moment_hand = zeros(row, 1);
%moment_cog = zeros(row, 1);
%moment_representative_points = zeros(row, 8);

%for i = 1:row
    %pos_vec_hand = hand_pos(i, :) - hand_pos(i, :);
    %pos_vec_hand = [pos_vec_hand, 0];
    %pos_vec_cog = cog_pos(i, :) - hand_pos(i, :);
    %pos_vec_cog = [pos_vec_cog, 0];

    %f_hand = [friction_hand(i, :), 0];
    %f_cog = [friction_cog(i, :), 0];

    %m_hand = cross(pos_vec_hand, f_hand);
    %m_hand = m_hand(3);
    %m_cog = cross(pos_vec_cog, f_cog);
    %m_cog = m_cog(3);

    %m_data = zeros(1, 8);
    %for j = 1:2:15
        %m_col = idivide(int32(j),2)+1;
        %pos_vec_rep  = [representative_pos(i, j), representative_pos(i, j+1)] - hand_pos(i,:);
        %pos_vec_rep = [pos_vec_rep, 0];
        %f_rep = [friction_representative_points(i, j), friction_representative_points(i, j+1), 0];
        %m_rep = cross(pos_vec_rep, f_rep);
        %m_rep = m_rep(3);
        %m_data(m_col) = m_rep;
    %end

    %moment_hand(i,:) = m_hand;
    %moment_cog(i, :) = m_cog;
    %moment_representative_points(i, :) = m_data;
%end

%save('use_data/time_typeA1.mat', 'time');
%save('use_data/hand_pos_typeA1.mat', 'hand_pos');
%save('use_data/cog_pos_typeA1.mat', 'cog_pos');
%save('use_data/representative_pos_typeA1.mat', 'representative_pos');
%save('use_data/dp_time_typeA1.mat', 'dp_time');
%save('use_data/dp_hand_pos_typeA1.mat', 'dp_hand_pos');
%save('use_data/dp_cog_pos_typeA1.mat', 'dp_cog_pos');
%save('use_data/dp_representative_pos_typeA1.mat', 'dp_representative_pos');
%save('use_data/velocity_hand_typeA1.mat', 'velocity_hand');
%save('use_data/velocity_cog_typeA1.mat', 'velocity_cog');
%save('use_data/velocity_representative_points_typeA1.mat', 'velocity_representative_points');
%save('use_data/unit_velocity_hand_typeA1.mat', 'unit_velocity_hand');
%save('use_data/unit_velocity_cog_typeA1.mat', 'unit_velocity_cog');
%save('use_data/unit_velocity_representative_points_typeA1.mat', 'unit_velocity_representative_points');
%save('use_data/friction_hand_typeA1.mat', 'friction_hand');
%save('use_data/friction_cog_typeA1.mat', 'friction_cog');
%save('use_data/friction_representative_points_typeA1.mat', 'friction_representative_points');
%save('use_data/moment_hand_typeA1.mat', 'moment_hand');
%save('use_data/moment_cog_typeA1.mat', 'moment_cog');
%save('use_data/moment_representative_points_typeA1.mat', 'moment_representative_points');