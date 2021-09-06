clc,clear
opts.path.bagfile_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/distance/bagfiles/";
opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/";
opts.path.event_name = "distance";
opts.data.simulation = 0;
opts.path.mat_path = opts.path.matfiles_root + opts.path.event_name + "/";


 % user parameters
opts.data.reload_matfiles = 0;
opts.data.num_scans = 1;
opts.target_num = 16;
mat_files = loadMatFilesFromFolder(opts.path.mat_path, '*Target*.mat');
num_targets = length(mat_files);


unprocessed_mat_files = loadMatFilesFromFolder(opts.path.mat_path, '*Target*.mat');

num_targets = 0;
for t = 1:length(unprocessed_mat_files)
    if contains(unprocessed_mat_files(t).file_name, "imgCorner")
        continue
    end
    num_targets = num_targets + 1;
    mat_files(num_targets) = unprocessed_mat_files(t);
end


num_targets = length(mat_files);
if num_targets == 0
    warning("No matfile is loaded, check data path: %s", opts.path.mat_path)
end

disp("Loading point cloud from .mat files")
pc = struct('point_cloud', cell(1,num_targets));
for t = 1:num_targets
    pc(t).point_cloud = loadPointCloud(mat_files(t).file_name);
end

disp("Pre-processing payload points...")
data = struct('point_cloud', cell(1,num_targets), ...
              'payload_points_h', cell(1,num_targets), ...
              'target_size', cell(1,num_targets)); % XYZIR 
for t = 1:num_targets
    data(t).name = mat_files(t).name;
    data(t).mat_file = mat_files(t).name;
    data(t).point_cloud = getPayloadWithIntensity(pc(t).point_cloud, 1, opts.data.num_scans);
    data(t).num_points = size(data(t).point_cloud, 2);
    data(t).payload_points_h = getPayload(pc(t).point_cloud, 1, opts.data.num_scans);
    data(t).target_scale = mat_files(t).target_scale;
end

figure_name = data(opts.target_num).name(1: strfind(data(opts.target_num).name,'mat')-2);
opts.path.bagfile = figure_name(1: strfind(data(opts.target_num).name,'-')-1) + ".bag";
disp("Data loaded!")

%% plot
project_name = "OptimalShape";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(2, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);

%%
[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
scatter3(axes_h, data(t).point_cloud(1,:), data(t).point_cloud(2,:), data(t).point_cloud(3,:), '')




















