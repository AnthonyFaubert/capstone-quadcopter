
load /tmp/data.mat

// gyro, ori, pErrs, mVals
rate = 50;
numPoints = size(gyro)(1);
time = 0:1/rate:(numPoints-1)/rate;

hold on;
plot(time, gyro(:, 1));
plot(time, gyro(:, 2));
plot(time, gyro(:, 3));
xlabel("Time (s)");
ylabel("degrees/s");
legend("x", "y", "z");
