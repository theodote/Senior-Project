function [s, unew, NRnew, cdnew, flag] = specsub(xframes, uold, NRold, cdold, threshold)

Xframes = fft(xframes,size(xframes,1),1);
u = zeros(size(xframes,1));

[flag, cdnew] = vad(Xframes(:,1), uold, threshold, cdold);

if flag > 0
    u = uold;
elseif flag <= 0
    u = mean(abs(Xframes),2);
end

subtscale = 1;
mutescale = 0.03;

Xmags = abs(Xframes);
Smags = Xmags - subtscale * u;  % the spectral subtraction!!!
Smag = Smags(:,1);  
Smags(Smags < 0) = 0;       % "half-wave rectification"
if flag > 0
    if Smag < NRold
        Smag = max(Smags,[],2); % residual noise reduction
    end
    NRnew = NRold;
elseif flag <= 0
    Smag = Smag * mutescale;
    NRnew = max(Smag, NRold);   % get new residue. S=N @ non speech!
end 

Xphase = angle(Xframes(:,1));
S = Smag .* exp(1j * Xphase);

s = real(ifft(S));
unew = u;
end