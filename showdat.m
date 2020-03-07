#!/usr/bin/octave -qf

SAMPLE_RATE = 100;

IMAGE_ASPECT_RATIO = [0 0 16 9]; # leave the first two zero and ignore them
IMAGE_SCALE = 1.5;
IMAGE_DPI = "-r150";
# b=blue, g=green, r=red, c=cyan, m=magenta, y=yellow, k=black, w=white
#keyboard();

args = argv();

function printUsage()
  print("Usage: ./showdat.m <debugData.mat> [headless <outFilesPrefix>]");
  quit();
end

outFile = 0;
if ((length(args) < 1) || (length(args) > 3))
  printUsage();
elseif (length(args) > 1)
  if (args{2} != 'headless')
    printUsage();
  endif
  outFile = args{3};
  graphics_toolkit gnuplot;
endif

load(args{1});

# gyro, ori, pErrs, mVals
numPoints = size(gyroRaw)(1);
time = 0:1/SAMPLE_RATE:(numPoints-1)/SAMPLE_RATE;
fft_f = (0:numPoints-1) * SAMPLE_RATE/numPoints;

CORRECTION_ANGLE = 50 * pi/180; # 50 degrees
gyroPitch = -gyroRaw(:, 1)*sin(CORRECTION_ANGLE) - gyroRaw(:, 2)*cos(CORRECTION_ANGLE);
gyroRoll = gyroRaw(:, 1)*cos(CORRECTION_ANGLE) + gyroRaw(:, 2)*sin(CORRECTION_ANGLE);

fft_gx = fft(gyroRaw(:, 1));
fft_gy = fft(gyroRaw(:, 2));
fft_gz = fft(gyroRaw(:, 3));
fft_gr = fft(gyroRoll);
fft_gp = fft(gyroPitch);
fft_proll = fft(pErrsRaw(:, 1));
fft_ppitch = fft(pErrsRaw(:, 2));
fft_pyaw = fft(pErrsRaw(:, 3));

cmdRoll = mVals(:, 4) - mVals(:, 2);
cmdPitch = mVals(:, 3) - mVals(:, 1);
cmdYaw = mVals(:, 1) - mVals(:, 3) + mVals(:, 2) - mVals(:, 4);

%{
# SingleStepPitch experiment
figure();
hold on;
plot(time, 50*mVals(:, 2));
plot(time, 50*mVals(:, 4));
plot(time, gyroPitch);
plot(time, gyroRaw(:, 1));
plot(time, gyroRaw(:, 2));
legend('m1*50', 'm3*50', 'pitch', 'x', 'y');
uiwait();
%}

if (outFile == 0)
  figure();
else
  figure('visible', 'off');
  set(gcf, 'PaperPosition', IMAGE_ASPECT_RATIO / IMAGE_SCALE);
endif
hold on;
title("Gyro data over time");
plot(time, gyroRoll, 'm');
plot(time, gyroPitch, 'b');
plot(time, gyroRaw(:, 3), 'k');
xlabel("Time (s)");
ylabel("Rotation velocity around axis (degrees/s)");
legend("roll (f_r(x,y))", "pitch (f_p(x,y))", "yaw (z)");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_gyro.png", outFile));
endif

if (outFile == 0)
  figure();
else
  clf;
endif
title("Frequency analysis of gyroscope data");
hold on;
plot(fft_f, abs(fft_gr), 'm');
plot(fft_f, abs(fft_gp), 'b');
plot(fft_f, abs(fft_gz), 'k');
xlabel("Frequency (Hz)");
ylabel("Magnitude of frequency component");
legend("roll (f_r(x,y))", "pitch (f_p(x,y))", "yaw (z)");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_gyro_fft.png", outFile));
endif

if (outFile == 0)
  figure();
else
  clf;
endif
hold on;
title("Proportional error over time");
plot(time, pErrsRaw(:, 1), 'm');
plot(time, pErrsRaw(:, 2), 'b');
plot(time, pErrsRaw(:, 3), 'k');
xlabel("Time (s)");
ylabel("Proportional error (unitless)");
legend("roll", "pitch", "yaw");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_pErrs.png", outFile));
endif

if (outFile == 0)
  figure();
else
  clf;
endif
title("Frequency analysis of proportional error");
hold on;
plot(fft_f, abs(fft_proll), 'm');
plot(fft_f, abs(fft_ppitch), 'b');
plot(fft_f, abs(fft_pyaw), 'k');
xlabel("Frequency (Hz)");
ylabel("Magnitude of frequency component");
legend("roll", "pitch", "yaw");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_pErrs_fft.png", outFile));
endif

if (outFile == 0)
  figure();
else
  clf;
endif
hold on;
title("PID motor value outputs over time");
plot(time, mVals(:, 1), 'm');
plot(time, mVals(:, 2), 'b');
plot(time, mVals(:, 3), 'k');
plot(time, mVals(:, 4), 'g');
%{
big = max(max(mVals));
small = min(min(mVals));
areas = (mVals > 1) | (mVals < 0);
for i = 1:length(mVals(:, 1))
  for j = 1:4
    
  end
end
if ( || () < 0))
  patch(time(areas(:, 2)), mVals(:, 2)(areas(:, 2)))
endif
%}
xlabel("Time (s)");
ylabel("Motor value (valid range [0, 1], actual values will be clipped to range)");
legend("motor 0", "motor 1", "motor 2", "motor 3");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_mVals.png", outFile));
endif

if (outFile == 0)
  figure();
else
  clf;
endif
hold on;
title("Throttle values over time");
plot(time, throttle, 'k');
## big = max(max(mVals));
## small = min(min(mVals));
## areas = (mVals > 1) | (mVals < 0);
## for i = 1:length(mVals(:, 1))
##   for j = 1:4
    
##   end
## end
## if ( || () < 0))
##   patch(time(areas(:, 2)), mVals(:, 2)(areas(:, 2)))
## endif
xlabel("Time (s)");
ylabel("Throttle (unitless, 0=0% thrust, 1=100% thrust)");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_throttle.png", outFile));
endif


if (outFile == 0)
  figure();
else
  clf;
endif
hold on;
title("PID debug graph");
plot(time, throttle);
plot(time, cmdRoll);
plot(time, cmdPitch);
plot(time, cmdYaw);
plot(time, gyroRoll / 50);
plot(time, gyroPitch / 50);
plot(time, gyroRaw(:, 3) / 50);
plot(time, pErrsRaw(:, 1));
plot(time, pErrsRaw(:, 2));
plot(time, pErrsRaw(:, 3));
xlabel("Time (s)");
legend('throttle (scale [0, 1])', 'roll command (scale [0, 1])', 'pitch command (scale [0, 1])', 'yaw command (scale [0, 2])', 'gyro roll (50 degrees/sec)', 'gyro pitch (50 degrees/sec)', 'gyro yaw (50 degrees/sec)', 'roll error (radians)', 'pitch error (radians)', 'yaw error (radians)');
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_throttle.png", outFile));
endif

if (outFile == 0)
  #keyboard();
  uiwait();
endif
