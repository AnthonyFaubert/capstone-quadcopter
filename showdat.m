
load data.mat

# gyro, ori, pErrs, mVals
rate = 50;
numPoints = size(gyro)(1);
time = 0:1/rate:(numPoints-1)/rate;

# b=blue, g=green, r=red, c=cyan, m=magenta, y=yellow, k=black, w=white

fft_gx = fft(gyro(:, 1));
fft_gy = fft(gyro(:, 2));
fft_gz = fft(gyro(:, 3));
fft_f = (0:numPoints-1) * rate/numPoints;

figure(1);
title("Gyro data over time");
hold on;
plot(time, gyro(:, 1), 'r');
plot(time, gyro(:, 2), 'b');
plot(time, gyro(:, 3), 'k');
xlabel("Time (s)");
ylabel("degrees/s");
legend("x", "y", "z");

figure(2);
title("Frequency analysis of gyroscope data");
hold on;
plot(fft_f, abs(fft_gx), 'r');
plot(fft_f, abs(fft_gy), 'b');
plot(fft_f, abs(fft_gz), 'k');
xlabel("Frequency (Hz)");
ylabel("Magnitude of frequency component");
legend("x", "y", "z");
