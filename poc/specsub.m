function [s, unew, NRnew, cdnew, flag] = specsub(xframes, yframes, uold, NRold, cdold, threshold)

Xframes = fft(xframes,size(xframes,1),1);
Xmags = abs(Xframes);
% plot(Xmags)
Yframes = fft(xframes,size(yframes,1),1);
Ymags = abs(Yframes);
% plot(Ymags)
u = zeros(size(xframes,1),1);

% REVIEW ORDER OF OPERATIONS from block diagram
[flag, cdnew] = vad(Xmags(:,1), uold, threshold, cdold);

if flag > 0
    u = uold;
elseif flag <= 0
    u = mean(abs(Xmags),2);
end
% plot(u)

subtscale = 4;
mutescale = 0.03;

Smags = Xmags - subtscale * u;  % the spectral subtraction
% plot(Smags)
Smags(Smags < 0) = 0;           % "half-wave rectification"
Smag = Smags(:,1);  
if flag > 0             % YES SPEECH
    for i = find(Smag < NRold)
        Smag(i) = max([Smag(i), Ymags(i,2:end)],[],2); % residual noise reduction
    end
    NRnew = NRold;
elseif flag <= 0        % NO SPEECH
    NRnew = max(Smag, NRold);   % get new residue. S=N @ non speech!
    Smag = Smag * mutescale;    % attenuare during non-activity
end 
% plot(Smags)

Xphase = angle(Xframes(:,1));
S = Smag .* exp(1j * Xphase);

s = real(ifft(S));
unew = u;
end