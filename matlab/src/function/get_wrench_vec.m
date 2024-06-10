% wrenchベクトルをただ描画するための関数

function [x, y, z] = get_wrench_vec(point)
    x = point(:, 1);
    y = point(:, 2);
    z = point(:, 3);
end