clear, clf;

% key parameters
framesize = 1024;
stepsize = framesize/2; % half-window overlap
hw = hann(framesize+1);
hw = hw(1:end-1);

% prepare raw audio
[stereo, fs] = audioread('hellowendy.wav');
raw = (stereo(:,1) + stereo(:,2)) / 2;
nsamples = length(raw);
nsamples = nsamples - mod(nsamples, framesize);
raw = raw(1:nsamples);      % truncate. all packets will be framesize

% optionally mess it up
raw = awgn(raw, 30);

% split into half-windows
steps = reshape(raw, stepsize, []);
nwindows = nsamples / framesize;
nframes = 2 * nwindows;

% get overlapping frames
inframes = zeros(framesize, nframes);
rebuilt = zeros(stepsize, nframes);

for i = 1 : nframes - 1
    inframes(:,i) = [steps(:,i) ; steps(:,i+1)] .* hw;
end
inframes(:,nframes) = [steps(:,nframes) ; zeros(stepsize,1)];
outframes = inframes(:,:);

%%% BEGIN PROCESSING

specnoise = fft(inframes(:,1),framesize ,1);    % both frequency domain
residual = zeros(framesize, 1);

voiceflags = zeros(stepsize, nframes);          % for debug plots
flag = 0;

threshold = 3;
countdown = 0;
avgs = 3;

for i = avgs : nframes
    [ ...
    outframes(:,i), ...
    specnoise, ...
    residual, ...
    countdown, ...
    flag ...
    ] = specsub( ...
        inframes(:,i:-1:i-avgs+1), ...
        outframes(:,i:-1:i-avgs+1), ...
        specnoise, ...
        residual, ...
        countdown, ...
        threshold ...
    );
    voiceflags(:, i) = flag * ones(stepsize, 1);
end

voiceflags = reshape(voiceflags, [], 1);

%%% ALL DONE

% reconstruct
rebuilt(:,1) = outframes(1:stepsize , 1);
for i = 2 : nframes
    rebuilt(:,i) = outframes(stepsize+1:end , i-1) + outframes(1:stepsize , i);
end
out = reshape(rebuilt, [], 1);

% plot & play
t = (1:nframes*stepsize)' / fs;
hold on
plot(t, raw, ":")
plot(t, out)
plot(t, raw-out)
plot(t, voiceflags)
legend('Raw', 'Processed', 'Difference', 'Voice flags')
xlim([0, nframes*stepsize/fs])
xlabel('Time (s)')
ylabel('Amplitude (float)')
title('Spectral subtraction demo')
sound(raw, fs)
pause(2.3)
sound(out, fs)