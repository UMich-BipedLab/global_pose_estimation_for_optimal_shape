clear, clc
run('/home/brucebot/workspace/test/CVPR17/setup.m')
addpath(genpath('/home/brucebot/workspace/lc-calibration/L1_relaxation'));
project_name = "OptimalShape";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(8, 1, project_name, clear_fig, 1, 30);
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);
disp("figure handles created")

%% user parameters
data = 2;
if data == 1
    pc_file = "optimal_shape-20          0          0-5          0        0.5.mat";
    downsample_name = "downsampled_points1.mat"; 
    opts.reuse_sampled = 1;
elseif data == 2
    pc_file = "optimal_shape-20         30         30-3.5          0        0.1.mat";
    downsample_name = "downsampled_points2.mat";
    opts.reuse_sampled = 1;
end

opts.max_iter = 15;
opts.downsample = 0;
opts.use_edge_points = 1;
opts.debug = 0;
opts.ncorrespondences.num_per_point = 1; % 1 ,2 ,4
opts.ncorrespondences.mode = 1; % 1: smallest dis 2: farthest dis


fig.save_figs = save_fig;

disp("user setting loaded")

%% tempalte
template = load("original_shape.mat");
rotatated_ideal = [300 0 0];
template.original_shape = ...
    moveByRPYXYZ(template.original_shape, rotatated_ideal, [0 0 0]);
template.original_shape = template.original_shape(1:3, :);
centroid = mean(template.original_shape, 2);
offset = [0 0 0];
ideal_angle_list = [0, 0, 0]; 
scale = 1;
[ideal_object_list, ideal_color_list] = createOptimalShape(...
    ideal_angle_list, -centroid', scale*template.original_shape);
disp("template loaded")

%% point cloud 
original_pc = load(pc_file); 
org_num_points = size(original_pc.object_list.points_mat, 2);


cur_pc = original_pc;
percentage = 0.1;


if opts.downsample 
    if opts.reuse_sampled == 0
        cur_pc.object_list = ...
            downsampleToPercentage(original_pc.object_list, percentage);
        save(downsample_name, 'original_pc')
    else
        load(downsample_name)
    end
end

if opts.use_edge_points
    edge_points = load('edge_points.mat');
    pc_points = reshape([edge_points.edge_points.Position].', 3,[]);
else
    pc_points = cur_pc.object_list.points_mat;
end
disp("pc loaded")


num_points = size(pc_points, 2);
fraction_left = num_points / org_num_points;


% transform current pc points
% fail
% 20 30 180;


% succeed
% 20,30,0
rpy = [0,0,0];
% rp = genRandomNumber(-40,40,[],2);
% heading = genRandomNumber(-60,60,[],1);
% rpy = [rp(1), rp(2), heading];
xyz = [0,0,0];
[cur_pc.object_list, ~] = createOptimalShape(...
    rpy, xyz, convertXYZstructToXYZmatrix(...
    cur_pc.object_list.object_vertices));

H_trans = constructHByRPYXYZMovingPoints(rpy, xyz);
cur_pc.object_list.points_mat = H_trans * ...
        convertToHomogeneousCoord(pc_points(1:3, :));
initial_pc =   cur_pc.object_list.points_mat; 
disp("pc transformed")    


original_pc_points = original_pc.object_list.points_mat;
[original_pc.object_list, ~] = createOptimalShape(...
    rpy, xyz, convertXYZstructToXYZmatrix(...
    original_pc.object_list.object_vertices));
original_pc.object_list.points_mat = H_trans * ...
        convertToHomogeneousCoord(original_pc_points(1:3, :));
    
disp("pc transformed")     



plane_correspondences = createOptimalShapePlaneCorrespondences(...
    original_pc.object_list.points_mat, ideal_object_list);
disp("formed original plane correspondences")

mode_list = [1,1,2,2,1,1,2,2,1,1];
mode_list = ones(1, opts.max_iter);

H_final = eye(4);
for iter = 1: opts.max_iter
    opts.ncorrespondences.mode = mode_list(iter);
    cur_pc.object_list.centroid = mean(cur_pc.object_list.points_mat(1:3, :), 2);
%     cur_pc.object_list.centroid
    cur_pc.object_list.centered_points = ...
        cur_pc.object_list.points_mat(1:3, :) - ...
        cur_pc.object_list.centroid;
    ideal_object_list = computeLineEquations(ideal_object_list);



    % find correspondences
    correspondences = findLineAsCorrespondence(opts, ...
        cur_pc, ideal_object_list, axes_handles(length(axes_handles)));

    if opts.debug == 1
        cur_axes = length(axes_handles)-1;
        [axes_h, fig_h] = getCurrentFigure(cur_axes, axes_handles, fig_handles);
        cla(axes_h)
        opts.plotting.model = 1;
        opts.plotting.plane = 1;
        opts.plotting.corrected_source = 0;
        opts.plotting.corrected2target = 0;
        opts.plotting.source2target = 1;
        opts.plotting.source_points = 1;
        opts.plotting.num_points = [];
        plotCorrespondences(opts, axes_handles, cur_axes, [], ...
            correspondences, Pose(), 1, "correspondences")
    end

    %%
    % opts.plotting.model = 1;
    % opts.plotting.plane = 0;
    % opts.plotting.corrected_source = 0;
    % opts.plotting.corrected2target = 0;
    % opts.plotting.source2target = 0;
    % opts.plotting.source_points = 0;
    % opts.plotting.num_points = [];
    % plotCorrespondences(opts, axes_handles, 4, [], ...
    %     correspondences, Pose(), 1, "test")
    % viewCurrentPlot(axes_h, "Quick viz", [-55, 10], 1)



    %% Solve
    s = 1;
    opts.quite = 1;
    opts.show_statistic = 1;
    out_t = solveRCQPAndSignedCost(opts, correspondences, [], s);
    out_t.original_cost = sum(abs(plane_correspondences.cost(Pose())));
    printOptimalShapeFittingResults(opts, iter, out_t);

    H = out_t.H.T;
    scaling = [out_t.s    0        0        0
               0        out_t.s    0        0
               0          0      out_t.s    0
               0          0        0        1];
    affine = H * scaling * constructHByRotationTranslation(eye(3), ...
        -cur_pc.object_list.centroid);
%     
%     transformed_pc = (affine) * converToHomogeneousCoord(...
%         cur_pc.object_list.centered_points(1:3, :));
    
    transformed_pc = (affine) * converToHomogeneousCoord(...
        cur_pc.object_list.points_mat(1:3, :));
    
     
    
    H_final = (affine) * H_final;
    transformed_original_pc = H_final * converToHomogeneousCoord(...
        original_pc.object_list.points_mat(1:3, :));
    estimated_vertices = H_final \ ...
        converToHomogeneousCoord(ideal_object_list.object_vertices_mat_h);
    % plotObjectsList(cur_axes, opts, LiDAR_opts, object_list)
    % plotOriginalAxisWithText(cur_axes, "LiDAR origin", eye(4), 0.5)

    %% Plottings
    [axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
    cla(axes_h)
    opts.simulate_lidar = 0;
    opts.arrow_length = 0.5;
    plotObjectsList(axes_h, opts, original_pc.LiDAR_opts, ideal_object_list);
    opts.simulate_lidar = 1;
    plotObjectsList(axes_h, opts, ...
        original_pc.LiDAR_opts, original_pc.object_list);
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    viewCurrentPlot(axes_h, "Initial Condition with Full PC", [-70, 10])

    
    if (iter == 1)
        [axes_h, fig_h] = getCurrentFigure(2, axes_handles, fig_handles);
        cla(axes_h)
        opts.arrow_length = 0.5;
        opts.simulate_lidar = 0;
        plotObjectsList(axes_h, opts, cur_pc.LiDAR_opts, ideal_object_list);
        opts.simulate_lidar = 0;
        plotObjectsList(axes_h, opts, ...
            cur_pc.LiDAR_opts, cur_pc.object_list);
        scatter3(axes_h, ...
            initial_pc(1, :), initial_pc(2, :), initial_pc(3, :), 'g.')
        viewCurrentPlot(axes_h, ...
            compose("Initial Condition \nwith Subsampled PC"), [-90,3])
    end

    
    [axes_h, fig_h] = getCurrentFigure(3, axes_handles, fig_handles);
    cla(axes_h)
    opts.arrow_length = 0.5;
    opts.simulate_lidar = 0;
    plotObjectsList(axes_h, opts, cur_pc.LiDAR_opts, ideal_object_list);
    opts.simulate_lidar = 0;
    plotObjectsList(axes_h, opts, ...
        cur_pc.LiDAR_opts, cur_pc.object_list);
    scatter3(axes_h, ...
        transformed_pc(1, :), transformed_pc(2, :),  transformed_pc(3, :), ...
        'b.')
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    viewCurrentPlot(axes_h, "Results with Subsampled PC", [-80,3])

    
    [axes_h, fig_h] = getCurrentFigure(4, axes_handles, fig_handles);
    cla(axes_h)
    opts.arrow_length = 0.5;
    opts.simulate_lidar = 0;
    plotObjectsList(axes_h, opts, original_pc.LiDAR_opts, ideal_object_list);
    opts.simulate_lidar = 1;
    plotObjectsList(axes_h, opts, ...
        original_pc.LiDAR_opts, original_pc.object_list);
    scatter3(axes_h, ...
        transformed_original_pc(1, :), transformed_original_pc(2, :),  transformed_original_pc(3, :), ...
        'b.')
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    scatter3(axes_h, ...
        estimated_vertices(1, :), ...
        estimated_vertices(2, :), ...
        estimated_vertices(3, :), 'ro', 'fill')
    viewCurrentPlot(axes_h, "Results with Full PC", [-90,3])
    
    
    [axes_h, fig_h] = getCurrentFigure(5, axes_handles, fig_handles);
    cla(axes_h)
    opts.simulate_lidar = 1;
    plotObjectsList(axes_h, opts, ...
        original_pc.LiDAR_opts, original_pc.object_list);
    scatter3(axes_h, ...
        estimated_vertices(1, :), ...
        estimated_vertices(2, :), ...
        estimated_vertices(3, :), 'ro', 'fill')
    viewCurrentPlot(axes_h, "Estimated Vertices with Full PC", [-90,3])
    
    
    cur_pc.object_list.points_mat = transformed_pc;
%     cur_axes = 5;
%     [axes_h, fig_h] = getCurrentFigure(cur_axes, axes_handles, fig_handles);
%     cla(axes_h)
%     opts.plotting.model = 0;
%     opts.plotting.plane = 1;
%     opts.plotting.corrected_source = 1;
%     opts.plotting.corrected2target = 0;
%     opts.plotting.source2target = 0;
%     opts.plotting.source_points = 0;
%     opts.plotting.num_points = [];
%     plotCorrespondences(opts, axes_handles, cur_axes, [], ...
%         correspondences, out_t.H, s, "Results")
%     opts.simulate_lidar = 0;
%     opts.arrow_length = 0.5;
%     plotObjectsList(axes_h, opts, cur_pc.LiDAR_opts, ideal_object_list);

   
    
    
    % opts.plotting.num_points = 1;
    % opts.plotting.model = 1;
    % opts.plotting.plane = 1;
    % opts.plotting.plane_boundary = 2;
    % opts.plotting.corrected_source = 1;
    % opts.plotting.corrected2target = 1;
    % opts.plotting.source2target = 1;
    % opts.plotting.source_points = 1;
    % plotCorrespondences(opts, axes_handles, cur_axes, [], ...
    %     correspondences, out_t.H, s, "test")
    % viewCurrentPlot(axes_h, "Results", [-55, 10], 1)
end
disp("=============================")
disp("======= Estimated RMS =======")
disp("=============================")
rpy
xyz
RMS = sqrt(norm(...
    estimated_vertices - original_pc.object_list.object_vertices_mat_h, ...
    'fro').^2/4)
disp("ALL Done")