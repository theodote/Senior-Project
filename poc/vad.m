function [voiceflag, uNew] = vad(frame, uOld, threshold, scheme)

X = fft(frame);
Xmag = abs(X);
u = fft(uOld);
umag = abs(u);
if scheme == 1
    T = 20*log10(mean(Xmag./umag)) - 4; % log mean ratio
elseif scheme == 2
    T = mean(20*log10(Xmag./umag));     % mean log ratio
else
    T = 0;
end

if T >= threshold
    uNew = uOld;
else
    uNew = frame;
end

voiceflag = T * ones(size(frame,1)/2, 1);

end