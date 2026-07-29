function out = Ampitude_frequency_plot(input)
    %% Fsfl
    Fs = 20;             % Sampling frequency (Hz)
    T = 1/Fs;            % Sampling period (s)
    L = length(input);   % Length of the force signal
    t = (0:L-1)*T;       % Time vector
    Y = fft(input);      % FFT of the force signal Fsfl

    P2 = abs(Y/L);       % Two-sided spectrum
    P1 = P2(1:L/2+1);    % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1);  % Double the amplitude for non-DC components

    f = Fs*(0:(L/2))/L;  % Frequency vector

    % Remove the first data point (usually the DC component at f = 0)
    f = f(2:end);         % Remove the first frequency (DC component)
    P1 = P1(2:end);       % Remove the first amplitude

    % Convert the amplitude spectrum to dB
    P1_dB = 10*log10(P1);

    % Find peaks in the amplitude spectrum
    [peaks, locs] = findpeaks(P1, f, 'MinPeakHeight', max(P1)*0.1); % Adjust 'MinPeakHeight' to control sensitivity

    % Set up the figure and plot
    out = figure;
    plot(f, P1_dB, 'color', 'k');   % Plot the frequency spectrum in dB
    title('Amplitude Spectrum');
    xlabel('Frequency (Hz)', 'Interpreter', 'latex');
    ylabel('Amplitude (dB)', 'Interpreter', 'latex');
    xlim([0 Fs/2]);      
    ylim([min(P1_dB) max(P1_dB)+5]);
    grid on;

    % Mark the peaks on the plot
    hold on;
    plot(locs, 10*log10(peaks), 'ro', 'MarkerFaceColor', 'r'); % Mark peaks with red circles

    % Add text labels near the peaks
    for i = 1:length(locs)
        text(locs(i), 10*log10(peaks(i)), sprintf('%.2f Hz\n%.2f dB', locs(i), 10*log10(peaks(i))), ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'LineWidth', 1, 'FontSize', 8);
    end

    hold off;
end
