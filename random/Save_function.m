function Save_function()
% Specify the desired folder path
folderPath = 'C:\MATLAB\vehicle Dynamics\nazanin mohandesi\results\RC2\signal_porcessing\';

% Check if the folder exists
if ~exist(folderPath, 'dir')
    % If the folder doesn't exist, create it
    mkdir(folderPath);
end

% Loop over all open figures
figures = findobj('Type', 'figure');  % Get handles of all open figures
for i = 1:length(figures)
    fig = figures(i);
    % Define the filename with a specific name or use the figure number
    filename = sprintf('%sfigure_%d.emf', folderPath, fig.Number);
    % Save the current figure as EMF file
    saveas(fig, filename, 'emf');
end

end