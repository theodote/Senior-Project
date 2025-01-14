function [s, uNnew, NRnew] = specsub(xframes, uNold, NRold, speechflag)

Xframes = fft(xframes,size(xframes,1),1);
uN = zeros(size(xframes,1));
if speechflag == 1
    uN = uNold;
elseif speechflag == 0
    uN = mean(abs(Xframes),2);
end

scale = 2;

Xmags = abs(Xframes);
Smags = Xmags - scale * uN; % the spectral subtraction!!!
Smag = Smags(:,1);  
Smags(Smags < 0) = 0;       % "half-wave rectification"
if speechflag == 1
    if Smag < NRold
        Smag = min(Smags,[],2); % residual noise reduction
    end
    NRnew = NRold;
elseif speechflag == 0
    Smag = Smag * 0.03;
    NRnew = max(Smag, NRold);   % get new residue. S=N @ non speech!
end 

Xphase = angle(Xframes(:,1));
S = Smag .* exp(1j * Xphase);

s = real(ifft(S));
uNnew = uN;
end