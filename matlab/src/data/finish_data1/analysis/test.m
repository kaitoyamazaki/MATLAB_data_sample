% サンプルデータの作成
x = linspace(0, 2*pi, 100);

% Figure1の作成
figure;

% 1行目のサブプロット
subplot(2, 2, 1);
y1 = sin(x);
plot(x, y1);
title('Sine Wave');
xlabel('x');
ylabel('sin(x)');
ylim([-0.005, 0.005]);

subplot(2, 2, 2);
y2 = cos(x);
plot(x, y2);
title('Cosine Wave');
xlabel('x');
ylabel('cos(x)');
ylim([-0.005, 0.005]);

% 2行目のサブプロット
subplot(2, 2, 3);
y3 = tan(x);
plot(x, y3);
title('Tangent Wave');
xlabel('x');
ylabel('tan(x)');
ylim([-0.005, 0.005]);

subplot(2, 2, 4);
y4 = exp(x);
plot(x, y4);
title('Exponential');
xlabel('x');
ylabel('exp(x)');
ylim([-0.005, 0.005]);

% Figure2の作成
figure;

% 1行目のサブプロット
subplot(2, 2, 1);
y5 = log(x);
plot(x, y5);
title('Logarithm');
xlabel('x');
ylabel('log(x)');
ylim([-0.005, 0.005]);

subplot(2, 2, 2);
y6 = x.^2;
plot(x, y6);
title('Square');
xlabel('x');
ylabel('x^2');
ylim([-0.005, 0.005]);

% 2行目のサブプロット
subplot(2, 2, 3);
y7 = sqrt(x);
plot(x, y7);
title('Square Root');
xlabel('x');
ylabel('sqrt(x)');
ylim([-0.005, 0.005]);

subplot(2, 2, 4);
y8 = x.^3;
plot(x, y8);
title('Cubic');
xlabel('x');
ylabel('x^3');
ylim([-0.005, 0.005]);
