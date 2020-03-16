function plot_func(titleName, xName, yName, dataList, dataOpt)
%{
    titleName: Name of the plot
    xName: X axis label
    yName: Y axis label
    dataList: Data of the main plot
        {plotData plotColor plotLegend;...}
    dataOpt: Subplot data
        {plotData plotColor plotLegend;...}
%}


[m, ~] = size(dataList);
if nargin < 3
    figure
    hold on

    for i = 1:m
        plot(dataList{i, 1}(:, 1), dataList{i, 1}(:, 2), dataList{i, 2}, 'DisplayName', dataList{i, 3}) 
    end
    legend
    title(titleName)
    xlabel(xName)
    ylabel(yName)

    hold off
else
    [n, ~] = size(dataOpt);
    figure
    
    subplot(1, 2, 1)
    hold on
    for i = 1:m
        plot(dataList{i, 1}(:, 1), dataList{i, 1}(:, 2), dataList{i, 2}, 'DisplayName', dataList{i, 3}) 
    end
    legend
    xlabel(xName)
    ylabel(yName)
    hold off
    
    subplot(1, 2, 2)
    hold on
    for i = 1:n
        plot(dataOpt{i, 1}(:, 1), dataOpt{i, 1}(:, 2), dataOpt{i, 2}, 'DisplayName', dataOpt{i, 3}) 
    end
    legend
    xlabel(xName)
    ylabel(yName)
    hold off
    suptitle(titleName)
    
end
