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
raw = awgn(raw, 20);

for i = 1 : nframes - 1
    frames(:,i) = [steps(:,i) ; steps(:,i+1)] .* hw;
end
frames(:,nframes) = [steps(:,nframes) ; zeros(stepsize,1)];

%%% DO THINGS TO FRAMES HERE FOR POC

specnoise = zeros(framesize);
residual = zeros(framesize);
voice = [zeros(nframes/2, 1) ; ones(nframes/2, 1)];
% voice = [zeros(fix(nframes*40000/nsamples), 1) ; ones(nframes-fix(nframes*40000/nsamples), 1)];

avgs = 3;
for i = avgs : nframes
    [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    % [frames(:,i), specnoise, residual] = specsub(frames(:,i:-1:i-avgs+1), specnoise, residual, voice(i));
    % plot(frames(:,1))
    % pause
end

%%% ALL DONE

rebuilt(:,1) = frames(1:stepsize , 1);
for i = 2 : nframes
    rebuilt(:,i) = frames(stepsize+1:end , i-1) + frames(1:stepsize , i);
end

out = reshape(rebuilt, [], 1);

clf
hold on
plot(raw)
plot(out)
plot(raw-out)
% spectrogram(out, hw, stepsize, framesize, fs, 'yaxis')
% sound(raw, fs)
% pause(2.07)
sound(out, fs)
% audiowrite('out.wav', out, fs)