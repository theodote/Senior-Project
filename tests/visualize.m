clear, clf;

framesize = 1024;
stepsize = framesize/2;     % half-window overlap
hw = hann(framesize+1);
hw = hw(1:end-1);

[stereo, fs] = audioread('final test.wav');
raw = (stereo(:,1) + stereo(:,2)) / 2;
% raw = stereo;
nsamples = length(raw);

% midpoint = 111614;
midpoint = 86000;

bypass = raw(1:midpoint);      % complete bypass
normal = raw(midpoint+1:end);    % normal operation

t = (1:nsamples)' / fs;
tb = t(1:midpoint);
tn = t(midpoint+1:end);

hold on
xlim([0, t(end)])
ylim([-0.25, 0.35])
plot(tb, bypass)
plot(tn, normal)
legend('"Complete bypass"', '"Normal operation"')
xlabel('Time (s)')
ylabel('Amplitude (float)')
title('Pale Sieve demonstration')
% sound(raw, fs)

% l = tiledlayout(2, 1, "TileSpacing", "compact");
% 
% ax1 = nexttile;
% spectrogram(raw, hw, stepsize, [], fs, 'yaxis')
% colormap(ax1, parula)
% title("Spectrogram")
% clim([-130, -40])
% 
% ax2 = nexttile;
% spectrogram(raw, hw, stepsize, [], fs, 'yaxis')
% colormap(ax2, bone)
% title("Compressed spectrogram")
% clim([-125, -105])