% シミュレーションで得られたデータを利用したアニメーションを作成する

% 初期化
clear;

% 検索ファイルパスの追加
addpath('.', '-end');
addpath('function', '-end');
addpath('data', '-end');

filepath_force_r = 'data/force_r_typeA.csv';
filepath_force_l = 'data/force_l_typeA.csv';

filepath_hand_pos_r = 'data/right_pos_typeA.csv';
filepath_hand_pos_l = 'data/left_pos_typeA.csv';

filepath_cop_pos = 'data/cop_pos_typeA.csv';
filepath_cog_pos = 'data/cog_pos_typeA.csv';

filepath_cop_ori = 'data/cop_ori_typeA.csv';
filepath_cog_ori = 'data/cog_ori_typeA.csv';

force_r = readmatrix(filepath_force_r);
force_l = readmatrix(filepath_force_l);

hand_pos_r = readmatrix(filepath_hand_pos_r);
hand_pos_l = readmatrix(filepath_hand_pos_l);

cop_pos = readmatrix(filepath_cop_pos);
cop_ori = readmatrix(filepath_cop_ori);

cog_pos = readmatrix(filepath_cog_pos);
cog_ori = readmatrix(filepath_cog_ori)