function plot_function(x, Y, titles)

    numPlots = length(Y); 


    fontSize = 12; 
    fontName = 'Arial';


    if numPlots == 1
        rows = 1; cols = 1;
    elseif numPlots == 2
        rows = 1; cols = 2;
    elseif numPlots == 3
        rows = 2; cols = 2;
    else
        rows = 2; cols = 2;
    end

    figure;
    

    for i = 1:numPlots
        subplot(rows, cols, i);
        plot(x, Y{i}, 'k', 'LineWidth', 1); 
        grid on;
        title(['\textbf{' titles{i} '}'], 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
        xlabel('\textbf{Time (s)}', 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
        ylabel('\textbf{Value}', 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
    end
end
