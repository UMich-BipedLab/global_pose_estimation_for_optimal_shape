clc, clear
% 1) run test_optimal_shape_exp with opts.dataset = 0, target_num_list = 99
%     and save('pose_illustration_sim.mat', 'original_pc', 'opts', 'ideal_object_list', 'gt_H_LT', 'T', 'est_H_LT') 
% 2) run test_optimal_shape_exp with opts.dataset = 6, target_num_list = 1
%     and save('pose_illustration_exp.mat', 'original_pc', 'opts', 'ideal_object_list', 'T', 'est_H_LT', 'estimated_vertices') 

exp = 1;
addpath(genpath("./data/"))
addpath(genpath("./paper_results/"))
if exp
    load('pose_illustration_exp.mat')
else
    load('pose_illustration_sim.mat')
end


project_name = "Pose Definition";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(1, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);


%%
[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
% opts.simulate_lidar = 0;
% opts.arrow_length = 0.5;
% plotObjectsList(axes_h, opts, original_pc.LiDAR_opts, ideal_object_list);
% opts.simulate_lidar = 1;
% plotObjectsList(axes_h, opts, ...
%     original_pc.LiDAR_opts, original_pc.object_list);

scatter3(axes_h, original_pc.object_list.points_mat(1, :), ...
    original_pc.object_list.points_mat(2, :), ...
    original_pc.object_list.points_mat(3, :), 'm.')
h1 = plotShape(axes_h, ideal_object_list.object_vertices_mat, 'k', 'k');
plotColoredOriginAxisWithText(axes_h, "Origin", eye(4), 0.3, 5, 0.05, [0, 0, 0.1], 'k')

if ~exp
    text(axes_h, 2, -0.4, 0.1, "$\mathcal{TP}$", 'Interpreter', 'latex', 'FontSize', axes_h.FontSize, 'Color', 'm')
    h0 = plotShape(axes_h, original_pc.object_list.object_vertices_mat, 'r', 'r');
    plotColoredOriginAxisWithText(axes_h, "", gt_H_LT, 0.5, 6, 0.1, [0.5, 0, -0.05], 'k');
    viewCurrentPlot(axes_h, "", [-50, 14])
%     saveCurrentPlot(fig_h(1), "PoseIllustration_sim", 'png')
else
    text(axes_h, 2, -0.3, 0.1, "$\mathcal{TP}$", 'Interpreter', 'latex', 'FontSize', axes_h.FontSize, 'Color', 'm')
    h0 = plotShape(axes_h, estimated_vertices, 'r', 'r');
    plotColoredOriginAxisWithText(axes_h, "", est_H_LT, 0.5, 8, 0.1, [0.5, 0, -0.05], 'k')
    viewCurrentPlot(axes_h, "", [-60, 10])
%     saveCurrentPlot(fig_h(1), "PoseIllustration_exp", 'png')
end
changeBackgroundColor(fig_h);


