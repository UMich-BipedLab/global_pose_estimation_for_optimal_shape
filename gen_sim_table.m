clear, clc
project_name = "SimTableForPaper";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(12, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);




%% load data
addpath(genpath("./data/"))
addpath(genpath("./paper_results/"))
table_path = "./paper_results/2021/simulation/";
table1_mat = "table_cell_noise_free.mat";
table2_mat = "table_cell_noise1.mat";
% table3_mat = "table_cell_noise2.mat";
table4_mat = "table_cell_noise3.mat";


table_data(1) = load(table_path + table1_mat);
table_data(2) = load(table_path + table2_mat);
% table_data(3) = load(table_path + table3_mat);
table_data(3) = load(table_path + table4_mat);

num_table = length(table_data);
num_data = length(table_data(1).table_cell)/2;


format 'bank'
% format 'short'
% show face-on data for table
noise_level = 2;
for j = 1:num_data
    table_data(noise_level).table_cell{j}
end

% show face-off data for table
for j = 21:2*num_data
    table_data(noise_level).table_cell{j}
end



legend_name = ["No Noise", "Noise 1", "Noise 2"];

%%
dist_t(num_table).face_on = [];
rmse_t(num_table).face_on = [];
trans_t(num_table).face_on = [];
rot_t(num_table).face_on = [];
rmse_t(num_table).face_on_percentage = [];
trans_t(num_table).face_on_percentage = [];


dist_t(num_table).face_off = [];
rmse_t(num_table).face_off = [];
trans_t(num_table).face_off = [];
rot_t(num_table).face_off = [];
rmse_t(num_table).face_off_percentage = [];
trans_t(num_table).face_off_percentage = [];


for i = 1:num_table
    for j = 1: num_data
        dis = table_data(i).table_cell{j}.target_distance;
        rmse = table_data(i).table_cell{j}.RMSE;
        trans = table_data(i).table_cell{j}.trans_error;
        rot = table_data(i).table_cell{j}.geo_rot;
        rmse_percentage = table_data(i).table_cell{j}.RMSE_percentage;
        trans_percentage = table_data(i).table_cell{j}.trans_error_percentage;
        
        dist_t(i).face_on = [dist_t(i).face_on, dis];
        rmse_t(i).face_on = [rmse_t(i).face_on, rmse];
        trans_t(i).face_on = [trans_t(i).face_on, trans];
        rot_t(i).face_on = [rot_t(i).face_on, rot];
        rmse_t(i).face_on_percentage = [rmse_t(i).face_on_percentage, rmse_percentage];
        trans_t(i).face_on_percentage = [trans_t(i).face_on_percentage, trans_percentage];
    end
    
    for j = num_data + 1: 2*num_data
        dis = table_data(i).table_cell{j}.target_distance;
        rmse = table_data(i).table_cell{j}.RMSE;
        trans = table_data(i).table_cell{j}.trans_error;
        rot = table_data(i).table_cell{j}.geo_rot;
        rmse_percentage = table_data(i).table_cell{j}.RMSE_percentage;
        trans_percentage = table_data(i).table_cell{j}.trans_error_percentage;
        
        dist_t(i).face_off = [dist_t(i).face_off, dis];
        rmse_t(i).face_off = [rmse_t(i).face_off, rmse];
        trans_t(i).face_off = [trans_t(i).face_off, trans];
        rot_t(i).face_off = [rot_t(i).face_off, rot];
        rmse_t(i).face_off_percentage = [rmse_t(i).face_off_percentage, rmse_percentage];
        trans_t(i).face_off_percentage = [trans_t(i).face_off_percentage, trans_percentage];
    end
end

colors = getColors(3);
[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), rmse_t(i).face_on(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end


viewCurrentPlot(axes_h, "Face-on: RMSE vs Distance", [], 0)
ylabel(axes_h, "RMSE [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
    saveCurrentPlot(fig_h, "Face-on-RMSE", 'png')
end


[axes_h, fig_h] = getCurrentFigure(2, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), trans_t(i).face_on(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Face-on: Translation vs Distance", [], 0)
ylabel(axes_h, "Translation Error [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Face-on-Translation", 'png')
end


[axes_h, fig_h] = getCurrentFigure(3, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), rot_t(i).face_on(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Face-on: Rotation vs Distance", [], 0)
ylabel(axes_h, "Rotation Error [deg]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Face-on-Rotation", 'png')
end



[axes_h, fig_h] = getCurrentFigure(4, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), rmse_t(i).face_off(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Challenging Angle: RMSE vs Distance", [], 0)
ylabel(axes_h, "RMSE [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Face-off-RMSE", 'png')
end



[axes_h, fig_h] = getCurrentFigure(5, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), trans_t(i).face_off(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Challenging Angle: Translation vs Distance", [], 0)
ylabel(axes_h, "Translation Error [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Face-off-Translation", 'png')
end


[axes_h, fig_h] = getCurrentFigure(6, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), rot_t(i).face_off(1:20), 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Challenging Angle: Rotation vs Distance", [], 0)
ylabel(axes_h, "Rotation Error [deg]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Face-off-Rotation", 'png')
end








%% all 
fig.save_figs = 0;
legend_name = ["Easy: No Noise", "Easy: Noise 1", "Easy: Noise 2", ...
    "Hard: No Noise", "Hard: Noise 1", "Hard: Noise 2"];

[axes_h, fig_h] = getCurrentFigure(7, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), rmse_t(i).face_on(1:20), '-o', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), rmse_t(i).face_off(1:20), '--^', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "RMSE vs Distance", [], 0)
ylabel(axes_h, "RMSE [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "RMSE-all", 'png')
end


[axes_h, fig_h] = getCurrentFigure(8, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), trans_t(i).face_on(1:20), '-o', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end

for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), trans_t(i).face_off(1:20), '--^', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Translation vs Distance", [], 0)
ylabel(axes_h, "Translation Error [m]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Translation-all", 'png')
end


[axes_h, fig_h] = getCurrentFigure(9, axes_handles, fig_handles);
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), rot_t(i).face_on(1:20), '-o', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end

for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), rot_t(i).face_off(1:20), '--^', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Rotation vs Distance", [], 0)
ylabel(axes_h, "Rotation Error [deg]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "Rotation-all", 'png')
end


[axes_h, fig_h] = getCurrentFigure(10, axes_handles, fig_handles);
cla(axes_h)
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), rmse_t(i).face_on_percentage(1:20), '-o', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end

for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), rmse_t(i).face_off_percentage(1:20), '--^', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end

viewCurrentPlot(axes_h, "RMSE vs Distance", [], 0)
ylabel(axes_h, "RMSE [%]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northwest')
if fig.save_figs
saveCurrentPlot(fig_h, "RMSE-percentage-all", 'png')
end


[axes_h, fig_h] = getCurrentFigure(11, axes_handles, fig_handles);
cla(axes_h)
h = [];
for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_on(1:20), trans_t(i).face_on_percentage(1:20), '-o', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end

for i = 1:num_table
    h1 = plot(axes_h, dist_t(i).face_off(1:20), trans_t(i).face_off_percentage(1:20), '--^', 'Color', colors{i}, 'LineWidth', 3);
    h = [h, h1];
end
viewCurrentPlot(axes_h, "Translation vs Distance", [], 0)
ylabel(axes_h, "Translation Error [%]")
xlabel(axes_h, "Distance [m]")
legend(axes_h, h, legend_name, 'Location', 'northeast')
if fig.save_figs
saveCurrentPlot(fig_h, "Translation-percentage-all", 'png')
end

