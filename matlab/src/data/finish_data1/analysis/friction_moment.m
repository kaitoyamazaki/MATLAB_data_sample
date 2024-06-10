% 指定した点で求めたモーメントを利用して合摩擦力モーメントを計算する

% 変数初期化
clear;

% 検索フォルダの追加
addpath('.', '-end');
addpath('res', '-end');

% .matファイルの読み込み
load_data = load('res/friction_moment.mat');
Friction_moment = load_data.friction_moment;

sum_friction_moment = [];

rows = size(Friction_moment, 1);

for i = 1:rows
    add_data = [];
    add_data(end+1) = Friction_moment(i, 1);
    
    sum_data = sum(Friction_moment(i, 2:9));
    %disp(sum_data)

    add_data(end+1) = sum_data;
    sum_friction_moment = [sum_friction_moment; add_data];
end

save('res/sum_friction_moment.mat', 'sum_friction_moment');