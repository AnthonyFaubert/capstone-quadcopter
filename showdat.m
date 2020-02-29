#!/usr/bin/octave -qf

SAMPLE_RATE = 50;

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
numPoints = size(gyro)(1);
time = 0:1/SAMPLE_RATE:(numPoints-1)/SAMPLE_RATE;
fft_f = (0:numPoints-1) * SAMPLE_RATE/numPoints;

fft_gx = fft(gyro(:, 1));
fft_gy = fft(gyro(:, 2));
fft_gz = fft(gyro(:, 3));

fft_proll = fft(pErrs(:, 1));
fft_ppitch = fft(pErrs(:, 2));
fft_pyaw = fft(pErrs(:, 3));

if (outFile == 0)
  figure();
else
  figure('visible', 'off');
  set(gcf, 'PaperPosition', IMAGE_ASPECT_RATIO / IMAGE_SCALE);
endif
hold on;
title("Gyro data over time");
plot(time, gyro(:, 1), 'm');
plot(time, gyro(:, 2), 'b');
plot(time, gyro(:, 3), 'k');
xlabel("Time (s)");
ylabel("Rotation velocity around axis (degrees/s)");
legend("x", "y", "z");
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
plot(fft_f, abs(fft_gx), 'm');
plot(fft_f, abs(fft_gy), 'b');
plot(fft_f, abs(fft_gz), 'k');
xlabel("Frequency (Hz)");
ylabel("Magnitude of frequency component");
legend("x", "y", "z");
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
plot(time, pErrs(:, 1), 'm');
plot(time, pErrs(:, 2), 'b');
plot(time, pErrs(:, 3), 'k');
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
ylabel("Motor value (valid range [0, 1], actual values will be clipped to range)");
legend("motor 0", "motor 1", "motor 2", "motor 3");
if (outFile != 0)
  print(IMAGE_DPI, sprintf("%s_mVals.png", outFile));
endif

if (outFile == 0)
  uiwait();
endif
