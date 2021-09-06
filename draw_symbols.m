clc, clear
% run test_optimal_shape_exp with opts.dataset = 6, target_num_list = 3
% and save('math_symbol_illustration.mat', 'occluded_points', 'opts', 'target_points', 'edge_points', 'estimated_vertices')
addpath(genpath("./data/"))
addpath(genpath("./paper_results/"))
load('math_symbol_illustration.mat')
project_name = "Math Symbol Illustration";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(1, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);


[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
if ~isempty(occluded_points)
    scatter3(axes_h, occluded_points(1, :), ...
        occluded_points(2, :), occluded_points(3, :), 20, 'go', 'fill')
end
scatter3(axes_h, target_points(1, :), ...
    target_points(2, :), target_points(3, :), 20, 'bo', 'fill')
scatter3(axes_h, ...
    edge_points(1, :), ...
    edge_points(2, :), ...
    edge_points(3, :), 100, 'mo', 'fill')
scatter3(axes_h, ...
    estimated_vertices(1, :), ...
    estimated_vertices(2, :), ...
    estimated_vertices(3, :), 100, 'ro', 'fill')
plot3(axes_h, ...
    [estimated_vertices(1, :), estimated_vertices(1, 1)], ...
    [estimated_vertices(2, :), estimated_vertices(2, 1)], ...
    [estimated_vertices(3, :), estimated_vertices(3, 1)], ...
    'r-', 'LineWidth', 3)
h1 = plotShape(axes_h, estimated_vertices, 'r', 'r');
text(axes_h, 8.1, -0.4, 0.3, "$\mathcal{TP} = \{\mathcal{X}_i\}_{i=1}^N$", 'Interpreter', 'latex', 'FontSize', axes_h.FontSize, 'Color', 'b')
text(axes_h, 8.1, -0.7, 0.1, "$\mathcal{EP} = \{\mathcal{E}_i\}_{i=1}^M$", 'Interpreter', 'latex', 'FontSize', axes_h.FontSize, 'Color', 'm')
text(axes_h, 8.1, -0.2, 0.4, "$X_{i+1}$", 'Interpreter', 'latex', 'FontSize', axes_h.FontSize, 'Color', 'm')
viewCurrentPlot(axes_h, "", [-90,0], 0);

% saveCurrentPlot(fig_h, "SymbolIllustration", 'png')
% reloadCurrentPlot