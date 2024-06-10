% wrench coneの下エッジを描画する関数
function plot_edge(point1, point2, point3)

    edge_x1 = [point1(1), point2(1)];
    edge_x2 = [point2(1), point3(1)];
    edge_x3 = [point3(1), point1(1)];

    edge_y1 = [point1(2), point2(2)];
    edge_y2 = [point2(2), point3(2)];
    edge_y3 = [point3(2), point1(2)];

    edge_z1 = [point1(3), point2(3)];
    edge_z2 = [point2(3), point3(3)];
    edge_z3 = [point3(3), point1(3)];

    plot3(edge_x1, edge_y1, edge_z1, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
    plot3(edge_x2, edge_y2, edge_z2, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
    plot3(edge_x3, edge_y3, edge_z3, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
end