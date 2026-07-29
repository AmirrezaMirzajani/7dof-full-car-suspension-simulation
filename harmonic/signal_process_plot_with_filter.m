function out = signal_process_plot_with_filter(input, footerText)
    Fs = 50;             % Sampling frequency (Hz)
    T = 1/Fs;            % Sampling period (s)
    L = length(input);    % Length of the signal
    t = (0:L-1)*T;       % Time vector

    % Design a low-pass FIR filter
    cutoff_freq = 5;     % Cutoff frequency at 5 Hz
    filter_order = 50;   % Filter length (number of coefficients)
    
    % Design the low-pass FIR filter
    b = fir1(filter_order, cutoff_freq/(Fs/2)); % Using fir1 for filter design
    filtered_signal = filter(b, 1, input);      % Apply the filter to the input signal

    % Compute the periodogram
    p1 = figure;

    % Create the first subplot for the periodogram
    subplot(3, 1, 1);  % 3 rows, 1 column, 1st plot
    [pxx, f] = periodogram(filtered_signal, [], [], Fs);
    plot(f, 10*log10(pxx), 'Color', 'k', 'LineWidth', 1.5); % Black color with thicker line
    title('Periodogram of Filtered Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for y-axis
    grid on;

    % Create the second subplot for the spectrogram
    subplot(3, 1, 2);  % 3 rows, 1 column, 2nd plot
    spectrogram(filtered_signal, 256, 250, 256, Fs, 'yaxis');
    view(50, 20); % Adjust view angle for better clarity
    shading interp;  % Interpolates the shading
    colorbar off;  % Disable colorbar if not needed
    title('Spectrogram of Filtered Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Time (s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    zlabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for z-axis

    % Create the third subplot for the power spectral density using pwelch
    subplot(3, 1, 3);  % 3 rows, 1 column, 3rd plot
    [pxx_pwelch, f_pwelch] = pwelch(filtered_signal, [], [], [], Fs); % Using pwelch to compute PSD
    plot(f_pwelch, 10*log10(pxx_pwelch), 'Color', 'b', 'LineWidth', 1.5); % Blue color for pwelch with thicker line
    title('Power Spectral Density (PSD) using pwelch', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
    xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex'); % Label for y-axis
    grid on;

    % Add footer text using annotation
  annotation('textbox', [0, 0, 1, 0.05], 'String', footerText, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 12, 'FontName', 'Arial', 'EdgeColor', 'none', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');


    out = p1; % Output the figure handle
end
