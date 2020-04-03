function plot_func(titleName, xName, yName, dataList, varargin)
%{
    titleName: Name of the plot
    xName: X axis label
    yName: Y axis label
    dataList: Data of the main plot
        {plotData plotColor plotLegend;...}
    dataOpt: Subplot data
        {plotData plotColor plotLegend;...}
%}

if length(varargin) > 0
    dataOpt = varargin{1};
else
    dataOpt = "";
end
   

[m, ~] = size(dataList);
if nargin < 5
    figure
    hold on

    for i = 1:m
        plot(dataList{i, 1}(:, 1), dataList{i, 1}(:, 2), dataList{i, 2}, 'DisplayName', dataList{i, 3}) 
    end
    legend
    title(titleName)
    xlabel(xName)
    ylabel(yName)
    grid on
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
    grid on
    hold off
    
    subplot(1, 2, 2)
    hold on
    for i = 1:n
        plot(dataOpt{i, 1}(:, 1), dataOpt{i, 1}(:, 2), dataOpt{i, 2}, 'DisplayName', dataOpt{i, 3}) 
    end
    legend
    xlabel(xName)
    ylabel(yName)
    grid on
    hold off
    suptitle(titleName)
    
end
