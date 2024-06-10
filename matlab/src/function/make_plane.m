% wrenchベクトルから, 平面を作成する関数

function [points_x, points_y, points_z] = make_plane(point1, point2)

    expected_number_of_rows = 100000;

    points = zeros(expected_number_of_rows, 3);
    count = 1;

    plane_coefficient = cross(point1, point2);

    for lamda = 0.0:0.01:1.0
        for mu = 0.0:0.01:1.0
            if (lamda + mu <= 1.0)
                point = lamda*point1 + mu*point2;

                if abs(plane_coefficient(1)*point(1) + plane_coefficient(2)*point(2) + plane_coefficient(3)*point(3)) < 1e-10
                    points(count, :) = point;
                    count = count + 1;
                end
            end
        end
    end

    remainder = rem(count, 5);

    points_x = reshape(points(1:count-remainder, 1), [], 5);
    points_y = reshape(points(1:count-remainder, 2), [], 5);
    points_z = reshape(points(1:count-remainder, 3), [], 5);

end