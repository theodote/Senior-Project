function [voiceflag, uNew] = vad(frame, uOld, threshold, scheme)

X = fft(frame);
Xmag = abs(X);
u = fft(uOld);
umag = abs(u);
if scheme == 1
    T = 20*log10(mean(Xmag./umag)) - 4; % log mean ratio, Boll
elseif scheme == 2
    T = mean(20*log10(Xmag./umag));     % mean log ratio, whoever???
else
    T = 0;
end

if T >= threshold
    uNew = uOld;
    T = 5;
else
    uNew = frame;
    T = 0;
end

voiceflag = T * ones(size(frame,1)/2, 1);

end