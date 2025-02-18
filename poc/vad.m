function [flag, cdnew] = vad(X, uold, threshold, cdold)

scheme = 1;
hangover = 50;

Xmag = abs(X);
umag = uold;

if scheme == 1      % so far superior!
    T = 20*log10(mean(Xmag./umag)) - 4; % log mean ratio, Boll
elseif scheme == 2
    T = mean(20*log10(Xmag./umag));     % mean log ratio, whoever???
else
    T = 0;
end

if T >= threshold   % voice
    flag = 1;
    cdnew = hangover;
else
    if cdold <= 0   % no more hangover!
        flag = 0;
        cdnew = 0;
    else            % still voice, hanging on
        flag = 1;
        cdnew = cdold - 1;
    end
end

end