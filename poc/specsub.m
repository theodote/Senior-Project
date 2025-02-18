function [s, unew, NRnew, cdnew, flag] = specsub(xframes, uold, NRold, cdold, threshold)

Xframes = fft(xframes,size(xframes,1),1);
Xmags = abs(Xframes);
% plot(Xmags)
u = zeros(size(xframes,1),1);

% REVIEW ORDER OF OPERATIONS from block diagram
[flag, cdnew] = vad(Xmags(:,1), uold, threshold, cdold);

if flag > 0
    u = uold;
elseif flag <= 0
    u = mean(abs(Xframes),2);   % BUG HERE!!! previous samples are nearly 0 due to mutescale. effectively /3. no wonder
end
% plot(u)

subtscale = 1;
mutescale = 0.03;

Smags = Xmags - subtscale * u;  % the spectral subtraction!!!
% plot(Smags)
Smag = Smags(:,1);  
Smags(Smags < 0) = 0;           % "half-wave rectification"
if flag > 0
    if Smag < NRold
        Smag = max(Smags,[],2); % residual noise reduction
    end
    NRnew = NRold;
elseif flag <= 0
    NRnew = max(Smag, NRold);   % get new residue. S=N @ non speech!
    Smag = Smag * mutescale;
end 
% plot(Smags)

Xphase = angle(Xframes(:,1));
S = Smag .* exp(1j * Xphase);

s = real(ifft(S));
unew = u;
end