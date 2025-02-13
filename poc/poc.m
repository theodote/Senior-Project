framesize = 1024;
stepsize = framesize/2;
hw = hann(framesize+1);
hw = hw(1:end-1);

[stereo, fs] = audioread('wyd.wav');
raw = (stereo(:,1) + stereo(:,2)) / 2;
raw = raw(9999:end);        % undo fade in for wyd.wav
nsamples = length(raw);
nsamples = nsamples - mod(nsamples, framesize);
raw = raw(1:nsamples);      % truncate. all packets will be framesize
steps = reshape(raw, stepsize, []);
nwindows = nsamples / framesize;
nframes = 2 * nwindows;

frames = zeros(framesize, nframes);
rebuilt = zeros(stepsize, nframes);

raw = raw + pinknoise(nsamples);
raw = awgn(raw, 20);

for i = 1 : nframes - 1
    frames(:,i) = [steps(:,i) ; steps(:,i+1)] .* hw;
end
frames(:,nframes) = [steps(:,nframes) ; zeros(stepsize,1)];

%%% DO THINGS TO FRAMES HERE FOR POC

specnoise = fft(frames(:,1),framesize ,1);    % FREQUENCY DOMAIN!!!
residual = zeros(framesize, 1); % FREQUENCY DOMAIN!!!
voice1 = zeros(stepsize, nframes);

threshold = 12;
countdown = 0;
flag = 0;

avgs = 3;
for i = avgs : nframes
    [frames(:,i), specnoise, residual, countdown, flag] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, countdown, threshold);
    voice1(:, i) = flag * ones(stepsize, 1);
end

voice1 = reshape(voice1, [], 1);

%%% ALL DONE

rebuilt(:,1) = frames(1:stepsize , 1);
for i = 2 : nframes
    rebuilt(:,i) = frames(stepsize+1:end , i-1) + frames(1:stepsize , i);
end

out = reshape(rebuilt, [], 1);

clf
hold on
% plot(raw*10, ":")
% plot(voice1)
plot(out)
plot(raw-out)
% spectrogram(raw, hw, stepsize, framesize, fs, 'yaxis')
% sound(raw, fs)
% pause(2.07)
% sound(out, fs)
audiowrite('out.wav', out, fs)