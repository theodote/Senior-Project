clear, clf;

% key parameters
framesize = 1024;
stepsize = framesize/2; % half-window overlap
hw = hann(framesize+1);
hw = hw(1:end-1);

% prepare raw audio
[stereo, fs] = audioread('hellowendy.wav');
raw = (stereo(:,1) + stereo(:,2)) / 2;
% raw = raw(9999:end);        % undo fade in for wyd.wav
nsamples = length(raw);
nsamples = nsamples - mod(nsamples, framesize);
raw = raw(1:nsamples);      % truncate. all packets will be framesize

% optionally mess it up
pinkscale = 0.005;
% raw = raw + pinkscale * pinknoise(nsamples);
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

%%% MAGIC HAPPENS HERE

specnoise = fft(inframes(:,1),framesize ,1);    % both frequency domain
residual = zeros(framesize, 1);

voiceflags = zeros(stepsize, nframes);          % for debug plots
flag = 0;

threshold = 2;

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

% plot
hold on
% plot(raw, ":")
% plot(voiceflags)
% plot(out)
% plot(raw-out)
spectrogram(out, hw, stepsize, framesize, fs, 'yaxis')
% sound(raw, fs)
% pause(2.1)
sound(out, fs)
% audiowrite('out.wav', out, fs)