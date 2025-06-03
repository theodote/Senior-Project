function [s, unew, NRnew, cdnew, flag] = specsub(xframes, yframes, uold, NRold, cdold, threshold)

Xframes = fft(xframes,size(xframes,1),1);
Xmags = abs(Xframes);
Yframes = fft(xframes,size(yframes,1),1);
Ymags = abs(Yframes);
u = zeros(size(xframes,1),1);

[flag, cdnew] = vad(Xmags(:,1), uold, threshold, cdold);

if flag > 0
    u = uold;
elseif flag <= 0
    u = mean(abs(Xmags),2);
end

subtscale = 4;
mutescale = 0.03;

Smags = Xmags - subtscale * u;  % the spectral subtraction
Smags(Smags < 0) = 0;           % "half-wave rectification"
Smag = Smags(:,1);  
if flag > 0             % YES SPEECH
    for i = find(Smag < NRold)
        Smag(i) = max([Smag(i), Ymags(i,2:end)],[],2); % residual noise reduction
        % seemingly ineffective
    end
    NRnew = NRold;
elseif flag <= 0        % NO SPEECH
    NRnew = max(Smag, NRold);   % get new residue. S=N @ non speech!
    Smag = Smag * mutescale;    % attenuare during non-activity
end 

Xphase = angle(Xframes(:,1));
S = Smag .* exp(1j * Xphase);

s = real(ifft(S));
unew = u;
end