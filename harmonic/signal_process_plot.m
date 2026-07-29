function out=signal_process_plot(input, footerText)
    Fs = 50;             % Sampling frequency (Hz)
    T = 1/Fs;            % Sampling period (s)
    L = length(input);    % Length of the signal
    t = (0:L-1)*T;       % Time vector

    % Compute the periodogram
    p1=figure;

    % Create the first subplot for the periodogram
    subplot(3, 1, 1);  % 3 rows, 1 column, 1st plot
    [pxx, f] = periodogram(input, [], [], Fs);
    plot(f, 10*log10(pxx), 'Color', 'k'); % Black color for the periodogram
    title('Periodogram of Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for z-axis
    grid on;

    % Create the second subplot for the Spectrogram
    subplot(3, 1, 2);  % 3 rows, 1 column, 2nd plot
    spectrogram(input, 256, 250, 256, Fs, 'yaxis');
    view(50,20)
    shading interp;  % Interpolates the shading
    colorbar off;  % Disable colorbar if not needed
    title('Spectrogram of the Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Time (s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    zlabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for z-axis

    % Create the third subplot for the Power Spectral Density using pwelch
    subplot(3, 1, 3);  % 3 rows, 1 column, 3rd plot
    [pxx_pwelch, f_pwelch] = pwelch(input, [], [], [], Fs); % Using pwelch to compute PSD
    plot(f_pwelch, 10*log10(pxx_pwelch), 'Color', 'b'); % Blue color for pwelch
    title('Power Spectral Density (PSD) using pwelch', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for z-axis
    grid on;

      annotation('textbox', [0, 0, 1, 0.05], 'String', footerText, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 12, 'FontName', 'Arial', 'EdgeColor', 'none', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');

    out = p1; % Output the figure handle
end
