clc, clear, close all

%%%%%%%%
% paths
%%%%%%%%
path_for_matlab_utils = "../../matlab_utils";
path_for_global_solver = "../global_sim3_solver";
path_for_lidar_simulator = "../lidar_simulator";



%%%%%%%%%%%%%%%%%%%
% Add dependencies
%%%%%%%%%%%%%%%%%%%
addpath(genpath(path_for_matlab_utils))
run(path_for_global_solver + "/setup.m")
addpath(genpath(path_for_global_solver))
addpath(genpath(path_for_lidar_simulator))
addpath(genpath("./data/"))
addpath(genpath("./paper_results/"))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Multi-cores for LiDAR Simulators
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If no pool, do not create new one.
poolobj = gcp('nocreate');
if isempty(poolobj)
    parpool('eight-threading');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path for data loading and saving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.save_path = "./data/Sep-2021/";
opts.load_path = "./data/Sep-2021/";
checkDirectory(opts.save_path)
checkDirectory(opts.load_path)


%%%%%%%%%%%%%%%%%%
% user parameters
%%%%%%%%%%%%%%%%%%
% 0: simulation
% 6: vary distance at FRB atrium (max 15 for target_num_list)
opts.dataset = 0; 




% In exp: which target to use. 
% In sim: which list of angle and translation to use 
%         check getAngleNTranslationList.m for reference
%         1-99: no noise
%         101-199: level 1 noise
%         201-299: level 2 noise
%         301-399: level 3 noise
%         401-499: level 4 noise
target_num_list = [101, 102]; % example for simulation
% target_num_list = [14, 15]; % example for experiments
% target_num_list = [15]; % example for one dataset


opts.target_occlusion = 0;
% for dataset 0, target 10, 20:22
% for dataset 6, target 1, 12:22
% for dataset 6, target 4, 19:22
% for dataset 6, target 10, 20:21
% for dataset 6, target 14, 21:22
opts.remove_rings = 21:22; 






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You usually don't need to change anything below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.postfix = "mocap-lidar-calibration";
opts.postfix = "mocap-lidar";
opts.filename = "original_shape.mat";
opts.regen_LiDAR_points = 0; % 0: if file exist, use it, or regenerate it
opts.use_best_shape = 1;
opts.plot = 1;
opts.save_plots = 0;
opts.save_results = 0;







table_cell = cell(1, length(target_num_list));
for target_num = 1:length(target_num_list)
opts.target_num = target_num_list(target_num);
opts = loadDatasetInfo(opts);



if opts.dataset~=0
    opts.save_path = "./paper_results/experiments/";
    opts.path.mat_path = opts.path.matfiles_root + opts.path.event_name + "/";
else
%     opts.save_path = "./paper_results/simulation/";
end
checkDirectory(opts.save_path)


%% LiDAR properties
[LiDAR_opts, boundary] = loadLiDARProperties(opts);


%% target orientation

if opts.use_best_shape && ~opts.data.simulation
    if opts.data.reload_matfiles
        % load everything in the folder with name containing '*Tag*mat'
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
        save(opts.save_path + opts.path.event_name + "-" + num2str(opts.data.num_scans) + "scans.mat", 'data', 'mat_files')
    else
        load(opts.save_path + opts.path.event_name + "-" + num2str(opts.data.num_scans) + "scans.mat")
    end
    figure_name = data(opts.target_num).name(1: strfind(data(opts.target_num).name,'mat')-2);
    opts.path.bagfile = figure_name(1: strfind(data(opts.target_num).name,'-')-1) + ".bag";
    disp("Data loaded!")
% 	figure(100)
%     clf(100)
%     scatter3(data(target_num).point_cloud(1,:), data(target_num).point_cloud(2,:), data(target_num).point_cloud(3,:), 'k.')
%     hold on
%     moved_points = moveByRPYXYZ(data(target_num).point_cloud(1:3, :), [-5 -20 0], [0 0 -1.2]);
%     data(target_num).point_cloud(1:3, :) = moved_points(1:3, :);
    object_list = assignFieldsToObjectList(opts.target_num, data);
%     h = scatter3(object_list.points_mat(1,:), object_list.points_mat(2,:), object_list.points_mat(3,:), 'b.');
%     axis equal
%     plotOriginalAxisWithText(cur_axes, "LiDAR origin", eye(4), 0.5)
    
    
    %% ideal target pose
    % target pose
    vertices = load(opts.load_path + opts.filename);
    rotatated_ideal = [300 0 0];
    vertices.original_shape = moveByRPYXYZ(vertices.original_shape, rotatated_ideal, [0 0 0]);
    vertices.original_shape = vertices.original_shape(1:3, :);
    
    scale =  object_list.target_scale;
    vertices.original_shape = scale*vertices.original_shape;
    centroid = mean(vertices.original_shape, 2);
    ideal_translation_list = -centroid'; 
    offset = [0 0 0];
    % translation_list = [0 0 0.5]; 
    ideal_angle_list = [0, 0, 0]; 
    
    [ideal_object_list, ideal_color_list] = createOptimalShape(ideal_angle_list, ideal_translation_list - offset, vertices.original_shape);
    [non_scale_ideal_object_list, ~] = createOptimalShape(ideal_angle_list, ideal_translation_list, vertices.original_shape);
    
elseif opts.use_best_shape && opts.data.simulation
    %% target pose
    vertices = load(opts.filename);
    rotatated_ideal = [300 0 0];
    vertices.original_shape = moveByRPYXYZ(vertices.original_shape, rotatated_ideal, [0 0 0]);
    vertices.original_shape = vertices.original_shape(1:3, :);
    [angle_list, translation_list] = getAngleNTranslationList(opts.target_position_list);
    [object_list, color_list] = createOptimalShape(angle_list, translation_list, vertices.original_shape);
    txt = printStructure("", angle_list) + "-";
    txt = printStructure(txt, translation_list, 0);
    
    if LiDAR_opts.properties.sensor_noise_enable
        if opts.target_num >= 100 && opts.target_num < 200
            noise_level = 1;
        elseif opts.target_num >= 200 && opts.target_num < 300
            noise_level = 2;
        elseif opts.target_num >= 300 && opts.target_num < 400
            noise_level = 3;
        elseif opts.target_num >= 400 && opts.target_num < 500
            noise_level = 4;
        else
            error("No such noise level %i", list_num)
        end
        name = "noise"+ num2str(noise_level) +"_optimal_shape-" + txt;
        %         save_fig_name = "noise1-neg_x3";
        save_fig_name = "noise"+num2str(noise_level)+"-pos_y3";
            
    else
        name = "optimal_shape-" + txt;
        save_fig_name = "ny1";
    end
    
elseif ~opts.use_best_shape && opts.data.simulation
    %% target pose
    angle_list = [30, 30, 20];
    translation_list = [5, 5 0.5];
%     translation_list = [-5, 5 0.5];
%     translation_list = [5, -5 0.5];
%     translation_list = [5, 5 -0.5];
    translation_list = [-5, -5 0.5]; %%
%     translation_list = [-5, 5 -0.5];
%     translation_list = [5, -5 -50.5]; %%
%     translation_list = [-5, -5, -0.5];
%     translation_list = [5, 0, 0];
    
    
    opts_obs.target_size = 1;
    opts_obs.polygon = 4;
    opts_obs.rpy = angle_list;
    opts_obs.xyz = translation_list;
    [object_list, color_list] = createDynamicScene(opts_obs);
    txt = printStructure("", angle_list) + "-";
    txt = printStructure(txt, translation_list, 0);
    name = "squares_shape-" + txt;
    
    %% ideal target pose
    opts_obs.rpy = [0, 0, 0];
    opts_obs.xyz = [0, 0, 0];
    [ideal_object_list, ideal_color_list] = createDynamicScene(opts_obs);
end


%% LiDAR Simulation
if opts.regen_LiDAR_points && opts.data.simulation
    [object_list, LiDAR_opts, boundary] = genLiDARPointsGivenObjectList(opts, object_list, LiDAR_opts, boundary);
    save(opts.save_path + name + ".mat", 'object_list', 'LiDAR_opts', 'boundary', 'color_list')
    disp("Generated LiDAR points on targets")
elseif ~opts.regen_LiDAR_points && opts.data.simulation
    file_to_be_load = opts.load_path + name + ".mat";
    if isfile(file_to_be_load)
        load(file_to_be_load)
        disp("Loaded LiDAR points on targets")
    else
        fprintf("Can't find the mat file: %s \nGenerating a new one...\n", file_to_be_load)
        [object_list, LiDAR_opts, boundary] = genLiDARPointsGivenObjectList(opts, object_list, LiDAR_opts, boundary);
        save(opts.save_path + name + ".mat", 'object_list', 'LiDAR_opts', 'boundary', 'color_list')
        disp("Generated LiDAR points on targets")
    end
end

%%

project_name = "OptimalShape";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(10, 1, project_name, clear_fig, 1, 30);
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);
disp("figure handles created")

%% user parameters
% data = 2;
% if data == 1
%     pc_file = "optimal_shape-20          0          0-5          0        0.5.mat";
%     downsample_name = "downsampled_points1.mat"; 
%     opts.reuse_sampled = 1;
% elseif data == 2
%     pc_file = "optimal_shape-20         30         30-3.5          0        0.1.mat";
%     downsample_name = "downsampled_points2.mat";
%     opts.reuse_sampled = 1;
% end




opts.reuse_sampled = 1;
opts.max_iter = 15;
opts.downsample = 0;
opts.use_edge_points = 1;
opts.debug = 0;
opts.ncorrespondences.num_per_point = 1; % 1 ,2 ,4
opts.ncorrespondences.mode = 1; % 1: smallest dis 2: farthest dis


fig.save_figs = save_fig;

disp("user setting loaded")

%% tempalte
if opts.data.simulation && opts.use_best_shape
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
elseif opts.data.simulation && ~opts.use_best_shape
    %% ideal target pose
    opts_obs.target_size = 1;
    opts_obs.polygon = 4;
    opts_obs.rpy = [0, 0, 0];
    opts_obs.xyz = [0, 0, 0];
    [ideal_object_list, ideal_color_list] = createDynamicScene(opts_obs);
end
disp("template loaded")

%% point cloud 
if opts.data.simulation
    pc_file = name + ".mat";
    original_pc = load(opts.load_path + pc_file); 
    
    org_num_points = size(original_pc.object_list.points_mat, 2);
else
    original_pc.object_list = object_list;
    original_pc.LiDAR_opts = LiDAR_opts;
    org_num_points = size(original_pc.object_list.points_mat, 2);
end



if org_num_points < 10 && opts.data.simulation
    file_to_delete = opts.load_path + pc_file;
    delete(file_to_delete)
    assert(org_num_points > 10, "the number of points are too less")
end




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
disp("pc loaded")
%% calculate the edge points
if opts.data.simulation
    [edge_points, target_points] = findEdgePointsFromLiDARPoints(opts, original_pc);
    occluded_points = [];
else
    edge_points = [];
    target_points = [];
    occluded_points = [];
    for ring = 1: opts.data.num_rings
        ring_points = original_pc.object_list.points_mat(:, original_pc.object_list.points_mat(5,:)==ring);
        if opts.target_occlusion && ismember(ring, opts.remove_rings)
            occluded_points = [occluded_points, ring_points];
            continue;
        end
        target_points = [target_points, ring_points];
        
        num_ring_points = size(ring_points, 2);
        if num_ring_points ~= 0
            [~, min_idx] = min(ring_points(2, :));
            [~, max_idx] = max(ring_points(2, :));
            edge_points = [edge_points, ring_points(:, min_idx)];
            if num_ring_points > 1
                edge_points = [edge_points, ring_points(:, max_idx)];
            end
        end
%         num_ring_points = size(ring_points, 2);
%         if num_ring_points > 0
%             edge_points = [edge_points, ring_points(1:3, 1)];
%             if num_ring_points > 1
%                edge_points = [edge_points, ring_points(1:3, end)];
%             end
%         end
        
    end
end

num_points = size(edge_points, 2);
fraction_left = num_points / org_num_points;


rpy = [0,0,0];
% rp = genRandomNumber(-40,40,[],2);
% heading = genRandomNumber(-60,60,[],1);
% rpy = [rp(1), rp(2), heading];

% origin
% 10,5,1;
xyz = [0,0,0];
if opts.data.simulation
    [cur_pc.object_list, ~] = createOptimalShape(...
        rpy, xyz, convertXYZstructToXYZmatrix(...
        cur_pc.object_list.object_vertices));
end


H_trans = constructHByRPYXYZMovingPoints(rpy, xyz);
cur_pc.object_list.points_mat = H_trans * ...
        convertToHomogeneousCoord(edge_points(1:3, :));
initial_pc =   cur_pc.object_list.points_mat; 
disp("pc transformed")    


original_pc_points = original_pc.object_list.points_mat;
if opts.data.simulation
    [original_pc.object_list, ~] = createOptimalShape(...
        rpy, xyz, convertXYZstructToXYZmatrix(...
        original_pc.object_list.object_vertices));
end
original_pc.object_list.points_mat = H_trans * ...
        convertToHomogeneousCoord(original_pc_points(1:3, :));
    
disp("pc transformed")     



original_plane_correspondences = createOptimalShapePlaneCorrespondences(...
    original_pc.object_list.points_mat, ideal_object_list);
disp("formed original plane correspondences")

mode_list = [1,1,2,2,1,1,2,2,1,1];
mode_list = ones(1, opts.max_iter);


% gt 
if opts.data.simulation
    [d, ~, transform] = procrustes(...
        original_pc.object_list.object_vertices_mat', ...
        ideal_object_list.object_vertices_mat', ...
        'scaling',false,'reflection',false);
    assert(d<1e-5, "gt is not correct: %.4f", d)
    original_pc.object_list.H = [transform.T' transform.c(1, :)'; 0 0 0 1];
    gt_vertices = original_pc.object_list.H * ...
            converToHomogeneousCoord(ideal_object_list.object_vertices_mat_h);
end


% compute line equations
ideal_object_list = computeLineEquations(ideal_object_list);

solver.H_final = eye(4); % projects the point cloud to the ideal frame so inv(H_final) reproject 
solver.H_previous = eye(4);
solver.cost_pre = 1e5;
solver.affine_pre = eye(4);
solver.computation_time = 0;

for iter = 1: opts.max_iter
    opts.ncorrespondences.mode = mode_list(iter);
%     cur_pc.object_list.centered_points = ...
%         cur_pc.object_list.points_mat(1:3, :);
%     plane_correspondences = createOptimalShapePlaneCorrespondences(...
%         cur_pc.object_list.centered_points, ideal_object_list);
%     % find correspondences
%     line_correspondences = findLineAsCorrespondence(opts, ...
%         cur_pc, ideal_object_list, axes_handles(length(axes_handles)));
% 
%     correspondences = [line_correspondences; plane_correspondences];
    
    t_start = cputime;
    cur_pc.object_list.centroid = mean(cur_pc.object_list.points_mat(1:3, :), 2);
    cur_pc.object_list.centered_points = ...
        cur_pc.object_list.points_mat(1:3, :) - ...
        cur_pc.object_list.centroid;
    
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
        viewCurrentPlot(axes_handles(cur_axes), "correspondences", [-55, 10], 1)
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
    solver.computation_time = solver.computation_time + (cputime - t_start);
    out_t.original_cost = sum(abs(original_plane_correspondences.cost(Pose())));
    printOptimalShapeFittingResults(opts, iter, out_t);
    solver.original_cost = out_t.original_cost;
    solver.optimized_cost = out_t.sum_cost;

    if solver.optimized_cost < solver.cost_pre
        H = out_t.H.T;
        scaling = [out_t.s    0        0        0
            0        out_t.s    0        0
            0          0      out_t.s    0
            0          0        0        1];
        affine = H * scaling * constructHByRotationTranslation(eye(3), ...
        -cur_pc.object_list.centroid);
        solver.affine_pre = H;
        solver.cost_pre = solver.optimized_cost;
    else
        affine = eye(4);
        scaling = 1;
    end
%         H = out_t.H.T;
%         scaling = [out_t.s    0        0        0
%             0        out_t.s    0        0
%             0          0      out_t.s    0
%             0          0        0        1];
%         affine = H * scaling * constructHByRotationTranslation(eye(3), ...
%         -cur_pc.object_list.centroid);
%         solver.affine_pre = H;
%         solver.cost_pre = solver.optimized_cost;
    
%     
%     transformed_pc = (affine) * converToHomogeneousCoord(...
%         cur_pc.object_list.centered_points(1:3, :));
    
    transformed_pc = (affine) * converToHomogeneousCoord(...
        cur_pc.object_list.points_mat(1:3, :));
    
     
    
    solver.H_final = (affine) * solver.H_final;
    
    
    
    
    transformed_original_pc = solver.H_final * converToHomogeneousCoord(...
        original_pc.object_list.points_mat(1:3, :));
    estimated_vertices = solver.H_final \ ...
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

    %%
    if (iter == 1)
        [axes_h, fig_h] = getCurrentFigure(2, axes_handles, fig_handles);
        cla(axes_h)
        opts.arrow_length = 0.5;
        opts.simulate_lidar = 0;
        plotObjectsList(axes_h, opts, cur_pc.LiDAR_opts, ideal_object_list);
        plotObjectsList(axes_h, opts, ...
            cur_pc.LiDAR_opts, cur_pc.object_list);
        opts.simulate_lidar = 1;
        plotObjectsList(axes_h, opts, ...
        original_pc.LiDAR_opts, original_pc.object_list);
        scatter3(axes_h, ...
            initial_pc(1, :), initial_pc(2, :), initial_pc(3, :), 'ro')
        viewCurrentPlot(axes_h, ...
            compose("Initial Condition \nwith Subsampled PC"), [-90,1])
    end

    
    [axes_h, fig_h] = getCurrentFigure(3, axes_handles, fig_handles);
    cla(axes_h)
    opts.arrow_length = 0.5;
    opts.simulate_lidar = 0;
    plotObjectsList(axes_h, opts, cur_pc.LiDAR_opts, ideal_object_list);
    opts.simulate_lidar = 0;
%     plotObjectsList(axes_h, opts, ...
%         cur_pc.LiDAR_opts, cur_pc.object_list);
%     scatter3(axes_h, ...
%         cur_pc.object_list.centered_points(1, :), ...
%         cur_pc.object_list.centered_points(2, :),  ...
%         cur_pc.object_list.centered_points(3, :), ...
%         'k.')
    scatter3(axes_h, ...
        transformed_original_pc(1, :), transformed_original_pc(2, :),  transformed_original_pc(3, :), ...
        'b.')
    scatter3(axes_h, ...
        transformed_pc(1, :), transformed_pc(2, :),  transformed_pc(3, :), ...
        'ro')
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    viewCurrentPlot(axes_h, "Results with Subsampled PC", [-90,2])
%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [axes_h, fig_h] = getCurrentFigure(4, axes_handles, fig_handles);
    cla(axes_h)
%     axes_h.reset;
%     hold(axes_h, 'on')
%     opts.simulate_lidar = 1;
%     plotObjectsList(axes_h, opts, ...
%         original_pc.LiDAR_opts, original_pc.object_list);
    
    if ~isempty(occluded_points)
        scatter3(axes_h, occluded_points(1, :), ...
            occluded_points(2, :), occluded_points(3, :), 20, 'go', 'fill')
    end
    scatter3(axes_h, target_points(1, :), ...
        target_points(2, :), target_points(3, :), 20, 'bo', 'fill')
%         scatter3(axes_h, original_pc.object_list.points_mat(1, :), ...
%             original_pc.object_list.points_mat(2, :), ...
%             original_pc.object_list.points_mat(3, :), 20, 'bo', 'fill')

%     scatter3(axes_h, ...
%         edge_points(1, :), ...
%         edge_points(2, :), ...
%         edge_points(3, :), 100, 'mo', 'fill')
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
    if opts.data.simulation
        h0 = plotShape(axes_h, gt_vertices, 'k', 'k');
        legend(axes_h, [h0, h1], ["gt", "est"], 'Location', 'northeast')
    end

%     viewCurrentPlot(axes_h, "Estimated Vertices with Full PC", [-90,0], 1);
    viewCurrentPlot(axes_h, "", [-90,0], 2);
% %     axis(axes_h, 'equal')
% %     reloadCurrentPlot(axes_h)
%     sim_dis = 2*opts.target_num;
%     xlim(axes_h, [sim_dis-0.5, sim_dis+0.5])
% saveCurrentPlot(fig_h, "Sim-" + num2str(sim_dis) + "m-front", "png")
%     zlim(axes_h, [-0.25, 0.65])
%     viewCurrentPlot(axes_h, "", [-3,12], 1);
%     xlim(axes_h, [sim_dis-0.5, sim_dis+0.5])
%     saveCurrentPlot(fig_h, "Sim-" + num2str(sim_dis) + "m-side", "png")
    
%     exp_dis = 2*(opts.target_num+1);
%     saveCurrentPlot(fig_h, "OccludedEXP-" + num2str(exp_dis) + "m-front", "png")
%     
%     viewCurrentPlot(axes_h, "", [-15, 1], 1);
%     saveCurrentPlot(fig_h, "OccludedEXP-" + num2str(exp_dis) + "m-side", "png")
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    
    
    [axes_h, fig_h] = getCurrentFigure(5, axes_handles, fig_handles);
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
    viewCurrentPlot(axes_h, "Results with Full PC", [-90,1])
    
    

    %% for video only
    [axes_h, fig_h] = getCurrentFigure(7, axes_handles, fig_handles);
    cla(axes_h)
    if ~isempty(occluded_points)
        scatter3(axes_h, occluded_points(1, :), ...
            occluded_points(2, :), occluded_points(3, :), 20, 'go', 'fill')
    end
    scatter3(axes_h, target_points(1, :), ...
        target_points(2, :), target_points(3, :), 20, 'bo', 'fill')
        opts.simulate_lidar = 0;
    opts.arrow_length = 0.5;
    h1 = plotShape(axes_h, ideal_object_list.object_vertices_mat, 'r', 'r');
%     plotObjectsList(axes_h, opts, original_pc.LiDAR_opts, ideal_object_list);
    plotOriginalAxisWithText(axes_h, "", eye(4), 1)
    viewCurrentPlot(axes_h, "", [-70, 10])
    %%
    

    if  norm(Log_SE3( affine \  solver.H_previous)) < 1e-5
        break;
    end
    
    
    cur_pc.object_list.points_mat = transformed_pc;
    solver.H_previous = affine;
    
    if ~opts.data.simulation
        [axes_h, fig_h] = getCurrentFigure(6, axes_handles, fig_handles);
        showBagFileImage(fig_h, opts.path.bagfile_root, opts.path.bagfile)
        title(axes_h, figure_name)
    end
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


%% print results
disp("=================================")
disp("======= Estimated Results =======")
disp("=================================")
fprintf("Spent %.2f seconds\n", solver.computation_time)
target_distance = norm(object_list.centroid);
if opts.data.simulation
    
    num_point_on_target = size(original_pc.object_list.points_mat, 2);
    quantization_error = deg2rad(LiDAR_opts.properties.az_resolution) * target_distance;
    
    RMSE = sqrt(norm(...
        estimated_vertices - original_pc.object_list.object_vertices_mat_h, ...
        'fro').^2/4);
    RMSE_percentage = RMSE / norm(object_list.centroid) * 100;
    
    
    gt_H_LT = original_pc.object_list.H
    est_H_LT = inv(solver.H_final)
    
    geo_dis = norm(Log_SE3(gt_H_LT / est_H_LT));
    geo_rot = rad2deg(norm(Log_SO3(gt_H_LT(1:3, 1:3)/est_H_LT(1:3, 1:3))));
    trans_error = norm(gt_H_LT(1:3, 4) - est_H_LT(1:3, 4));
    trans_error_percentage = trans_error / norm(object_list.centroid) * 100;
    
%     geo_dis = norm(Log_SE3(object_list.H_inv/solver.H_final));
%     geo_rot = rad2deg(norm(Log_SO3(object_list.H_inv(1:3, 1:3)/solver.H_final(1:3, 1:3))));
%     trans_error = norm(object_list.H_inv(1:3, 4) - solver.H_final(1:3, 4));
%     trans_error_percentage = trans_error / norm(object_list.centroid) * 100;
    T = table(target_distance, quantization_error, num_point_on_target, ...
        RMSE, RMSE_percentage, trans_error,  trans_error_percentage, geo_rot)
    
    fprintf("Number of Points on the Target: %i\n", num_point_on_target)
    fprintf("Quantization Error: %i\n", quantization_error)
    fprintf("Geodesic Distance on SE(3) (norm(Log(H_gt * inv(H_est)))): %.4f \n", geo_dis)
    fprintf("Geodesic Distance on SO(3) (norm(Log(R_gt * inv(R_est)))): %.4f [deg]\n", geo_rot)
    fprintf("Translation Error: %.4f [m]\n", trans_error)
    fprintf("Percentage of Translation Error: %.4f [%%] \n", trans_error_percentage)
    fprintf("Percentage of Vertices Error: %.4f [%%] \n", RMSE_percentage)
    fprintf("RMSE: %.2f [m]\n", RMSE)
else
    original_cost = solver.original_cost;
    optimized_cost = solver.optimized_cost;
    est_H_LT = inv(solver.H_final);
    num_point_on_target = size(original_pc.object_list.points_mat, 2);
    quantization_error = deg2rad(LiDAR_opts.properties.az_resolution) * target_distance;
    fprintf("Number of Points on the Target: %i\n", num_point_on_target)
    fprintf("Target Distance: %.2f [m]\n",  target_distance)
    fprintf("original_cost: %.2f [m]\n", solver.original_cost)
    fprintf("optimized_cost: %.2f [m]\n", solver.optimized_cost)
    varNames = {'Name', 'Distance','QuantizationError','NumberOfPoints', 'EstVertices', 'EstH', 'OriginalCost', 'OptimizedCost', 'points', 'template'};
%     T = table();
    T = table({figure_name}, target_distance, quantization_error, num_point_on_target, ...
        {estimated_vertices}, {est_H_LT}, ...
        original_cost, optimized_cost, ...
        {original_pc.object_list.points_mat}, ...
        {ideal_object_list.object_vertices_mat_h}, 'VariableNames', varNames);
end
table_cell{target_num} = T;

    %% save data
    if opts.data.simulation && opts.save_results
        cvs_name = "results_noise"+opts.postfix+".csv";
        if ~isfile(cvs_name)
            fid = fopen(cvs_name, 'A' );
            fprintf(fid, 'x, y, z, r, p, h, rmse, rmse_per, geo_dis, geo_rot, trans_error, trans_error_per\n');
        else
            fid = fopen(cvs_name, 'A' );
        end
        fprintf(fid, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', ...
            translation_list(1), translation_list(2), translation_list(3), ...
            angle_list(1), angle_list(2), angle_list(3), ...
            RMSE, RMSE_percentage, geo_dis, geo_rot, ...
            trans_error, trans_error_percentage);
        fclose( fid );
    end
    
    
disp('Pess to continue')
pause
end
disp("=================================")
if opts.save_results
    save("table_cell-"+ opts.postfix +".mat", 'table_cell')
end
disp("ALL Done")
