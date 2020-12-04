#!/usr/bin/octave-cli
pkg load 'statistics';
args = argv();
ts_diff_threshold = 0.1;

function print_usage()
  printf("Usage: plot_ts_diff.m <message.csv>\n");
  printf("Example: plot_ts_diff.m vicon.csv\n");
endfunction

% Run
if length(args) != 1
  printf("Invalid arguments!\n");
  print_usage();
  return;
endif

% Analyze time differences between consecutive timestamps
data = csvread(args{1}, 1, 0);

% -- From first timestamp
secs = data(1, 1);
nsecs = data(1, 2) * 1e-9;
t0 = secs + nsecs;

% -- From last timestamp
secs = data(end, 1);
nsecs = data(end, 2) * 1e-9;
tlast = secs + nsecs;

% -- Calculate time difference between consecutive timestamps
ts_diff = [];
ts_prev = 0;

time = []
for k = 2:rows(data)
  secs = data(k, 1);
  nsecs = data(k, 2) * 1e-9;
  ts = (secs + nsecs) - t0;

  ts_diff_k = (ts - ts_prev);
  if (ts_diff_k < 0)
    ts_prev
    ts
    ts_diff_k
    printf("Invalid timestamp diff!");
    break;
  end

  ts_diff = [ts_diff; ts_diff_k];
  time = [time; ts];
  ts_prev = ts;
endfor

% -- Calculate number of outliers
nb_outliers = 0;
for i = 1:length(ts_diff)
  if ts_diff(i) > ts_diff_threshold
    nb_outliers += 1;
  end
endfor

% window_size = 1;
% window = [];
% rate = [];
% for i = 1:length(ts_diff)-window_size
%   window = [window; 1.0 / ts_diff(i)];
%   if length(window) == window_size
%     rate = [rate; mean(window)];
%     window = window(2:end);
%   endif
% endfor
%
% y = fft(rate);
% plot(rate, y);
% ginput();

% -- Show metrics
printf("time length:       %.2f [s]\n", tlast - t0);
printf("ts_diff_threshold: %d [s]\n", ts_diff_threshold);
printf("nb_outliers:       %d\n", nb_outliers);
printf("ts_diff:\n");
printf("  mean:   %f [s]\n", mean(ts_diff));
printf("  median: %f [s]\n", median(ts_diff));
printf("  var:    %f [s]\n", var(ts_diff));
printf("  stddev: %f [s]\n", sqrt(var(ts_diff)));
printf("  min:    %f [s]\n", min(ts_diff));
printf("  max:    %f [s]\n", max(ts_diff));

% -- Show plots
% figure(1);
% scatter(1:length(ts_diff), ts_diff);
% xlabel("i-th timestamp");
% ylabel("Timestamp diff [s]");
%
% figure(2);
% hold on;
% boxplot(ts_diff);

% figure()
% plot(1:length(rate), rate);
% ginput()

figure(3);
hold on;
plot(time, 1 ./ ts_diff);
% xlim([min(time), max(time)]);
xlim([min(time), 1.0]);
ylim([0, 150.0]);
xlabel("Time [s]")
ylabel("Rate [hz]")

ginput();
