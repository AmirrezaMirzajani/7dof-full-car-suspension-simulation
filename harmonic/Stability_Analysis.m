function out = Stability_Analysis(input)
    Fs = 20; % Sampling frequency (Hz)
    T = 1/Fs; % Sampling period (s)
    L = length(input); % Length of the input signal
    t = (0:L-1)*T; % Time vector

    % FFT of the input signal
    Y = fft(input);

    % Two-sided spectrum
    P2 = abs(Y/L);

    % Single-sided spectrum
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1); % Double the amplitude for non-DC components

    f = Fs*(0:(L/2))/L; % Frequency vector

    % Compute phase (in degrees)
    phase = angle(Y(1:L/2+1)) * (180/pi); % Convert phase to degrees

    % Find indices where magnitude is above 0 dB and phase is close to 180 degrees
    unstable_idx = find(P1 > 1 & abs(phase - 180) < 10); % P1 > 1 means magnitude > 0 dB, phase ~ 180 degrees

    % Plot the magnitude and phase
    figure;
    subplot(2, 1, 1);
    plot(f, 20*log10(P1), 'k'); % Plot magnitude in dB
    title('Magnitude Spectrum');
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    grid on;

    subplot(2, 1, 2);
    plot(f, phase, 'k'); % Plot phase in degrees
    title('Phase Spectrum');
    xlabel('Frequency (Hz)');
    ylabel('Phase (Degrees)');
    grid on;

    % Mark unstable points on the plot
    hold on;
    plot(f(unstable_idx), 20*log10(P1(unstable_idx)), 'ro'); % Mark unstable points
    hold off;

    % Output figure handle
    out = gcf;
end
