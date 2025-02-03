framesize = 1024;
stepsize = framesize/2;
hw = hann(framesize+1);
hw = hw(1:end-1);

[stereo, fs] = audioread('hellowendy.wav');
raw = (stereo(:,1) + stereo(:,2)) / 2;
nsamples = length(raw);
nsamples = nsamples - mod(nsamples, framesize);
raw = raw(1:nsamples);      % truncate. all packets will be framesize
steps = reshape(raw, stepsize, []);
nwindows = nsamples / framesize;
nframes = 2 * nwindows;

frames = zeros(framesize, nframes);
rebuilt = zeros(stepsize, nframes);

% raw = raw + pinknoise(nsamples);
% raw = awgn(raw, 20);

for i = 1 : nframes - 1
    frames(:,i) = [steps(:,i) ; steps(:,i+1)] .* hw;
end
frames(:,nframes) = [steps(:,nframes) ; zeros(stepsize,1)];

%%% DO THINGS TO FRAMES HERE FOR POC

% specnoise = zeros(framesize, 1);
specnoise = frames(:,1);
residual = zeros(framesize, 1);
% voice = [zeros(nframes/2-5, 1) ; ones(nframes/2+5, 1)];
% voice = [zeros(fix(nframes*40000/nsamples), 1) ; ones(nframes-fix(nframes*40000/nsamples), 1)];
voice1 = zeros(stepsize, nframes);
voice2 = zeros(stepsize, nframes);

threshold = 3.4;

avgs = 3;
for i = avgs : nframes
    [voice1(:, i), specnoise] = vad(frames(:,i), specnoise, threshold, 1);
    [voice2(:, i), specnoise] = vad(frames(:,i), specnoise, threshold, 2);
    % [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    % [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    % [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    % plot(frames(:,1))
    % pause
end

voice1 = reshape(voice1, [], 1);
voice2 = reshape(voice2, [], 1);

%%% ALL DONE

rebuilt(:,1) = frames(1:stepsize , 1);
for i = 2 : nframes
    rebuilt(:,i) = frames(stepsize+1:end , i-1) + frames(1:stepsize , i);
end

out = reshape(rebuilt, [], 1);

clf
hold on
plot(raw*10, ":")
plot(voice1)
plot(voice2)
% plot(out)
% plot(raw-out)
% spectrogram(raw, hw, stepsize, framesize, fs, 'yaxis')
% sound(raw, fs)
% pause(2.07)
% sound(out, fs)
% audiowrite('out.wav', out, fs)