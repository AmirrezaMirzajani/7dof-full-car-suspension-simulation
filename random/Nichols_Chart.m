function out=Nichols_Chart(input)
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

    % Plot the Nichols Chart (Magnitude vs Phase)
    magnitude = 20*log10(P1); % Convert to dB scale
    phase = angle(Y(1:L/2+1)) * (180/pi); % Convert phase to degrees

    % Create a figure
    figure;
    plot(magnitude, phase, 'k'); % Plot magnitude vs phase
    title('Nichols Chart for System');
    xlabel('Magnitude (dB)');
    ylabel('Phase (Degrees)');
    grid on;

    out = gcf;
end
