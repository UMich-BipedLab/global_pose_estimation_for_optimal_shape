% This script does the following


clc, clear
Bruce = 1;
if Bruce
    run('/home/brucebot/workspace/test/CVPR17/setup.m')
    addpath(genpath('/home/brucebot/workspace/lc-calibration/L1_relaxation'));
    poolobj = gcp('nocreate');
    % If no pool, do not create new one.
    if isempty(poolobj)
        parpool('eight-threading');
    end

else
    run('/home/lee/21summer/convex_relaxation_sim3/setup.m');
    addpath(genpath("/home/lee/21summer/matlab_utils"));
    loadLibraries(2);
    rmpath("/home/lee/21summer/extrinsic_lidar_camera_calibration/extrinsic_utils/computeCost/")
    addpath(genpath('/home/lee/21summer/L1_relaxation'));
end
% just common this to make code work


% previous settings
% opts.save_path = "./results/experiments";
% opts.save_path = "./results/simulation";
% opts.load_path = "./";
% opts.filename = "original_shape.mat";

% setting on 2021-4-27
% opts.save_path = "./results/April-2021/experiments";
opts.save_path = "./data/July-2021/";
opts.load_path = "./data/July-2021/";
opts.filename = "original_shape.mat";


% user parameters
opts.regen_LiDAR_points = 1;
opts.use_best_shape = 1;
opts.plot = 1;
opts.save_plots = 0;

% 0: simulation
% 1: lab 
% 2: wavefield
% 3: robotics building
% 4: calibration
% 5: old data (no images)
opts.dataset = 0; 
opts.target_num = 8;

if opts.dataset == 0 
    opts.debug = 0;
    opts.verbose.output_level = 0;
    opts.verbose.selective = 0;
    opts.data.simulation = 1;
    opts.target_position_list = 105; % which angle/translation list to use
    
    if opts.target_position_list >= 100
        opts.sensor_noise_enable = 1;
        opts.sensor_noise_level = [0.0300, 0.2000, 0.2000];
    else
        opts.sensor_noise_enable = 0;
    end
    
elseif opts.dataset == 1
    opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/";
    opts.path.bagfile_root = "/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/";
    opts.path.event_name = "lab";
    opts.data.simulation = 0;
    
     % user parameters
    opts.data.reload_matfiles = 0;
    opts.data.num_scans = 1;

elseif opts.dataset == 2
    opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/";
    opts.path.bagfile_root = "/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/";
    opts.path.event_name = "wavefield";
    opts.data.simulation = 0;
    
     % user parameters
    opts.data.reload_matfiles = 0;
    opts.data.num_scans = 1;
    
elseif opts.dataset == 3
    opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/";
    opts.path.bagfile_root = "/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/";
    opts.path.event_name = "roboticsBuildings";
    opts.data.simulation = 0;
    
     % user parameters
    opts.data.reload_matfiles = 0;
    opts.data.num_scans = 1;
    
elseif opts.dataset == 4
    opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/calibration/";
    opts.path.bagfile_root = "/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/";
    opts.path.event_name = "QuanCalibration";
    opts.data.simulation = 0;
    
     % user parameters
    opts.data.reload_matfiles = 0;
    opts.data.num_scans = 1;    
    
elseif opts.dataset == 5
    opts.path.matdata_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/";
    opts.path.event_name = "new_target_mat_files_08052020";
    opts.data.simulation = 0;
    
    % user parameters
    opts.data.reload_matfiles = 0;
    opts.data.num_scans = 1;
end

if opts.dataset~=0
    opts.save_path = "./paper_results/experiments/";
    opts.path.mat_path = opts.path.matfiles_root + opts.path.event_name + "/";
else
%     opts.save_path = "./paper_results/simulation/";
end
checkDirectory(opts.save_path)


%% LiDAR properties
% LiDAR properties
LiDAR_opts.properties.range = 150;
LiDAR_opts.properties.return_once = 0;
LiDAR_opts.pose.centriod = [0 0 0];
LiDAR_opts.pose.rpy = [180 0 0]; % deg (roll pitch yaw)
LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);
LiDAR_opts.properties.mechanics_noise_model = 0; 
LiDAR_opts.properties.sensor_noise_enable = opts.sensor_noise_enable;
LiDAR_opts.properties.rpm = 1200; % 300, 600, 900, 1200
LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
[LiDAR_opts.properties.ring_elevation, ...
LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties, 'ring_', LiDAR_opts.properties.beam);
LiDAR_opts.mechanism.types = ["rotating-head", "solid-state"];
LiDAR_opts.mechanism.type = 1;

% Workspace boundary
boundary.x = [160, -160];
boundary.y = [160, -160];
boundary.z = [160, -160];
boundary.vertices = createBoxVertices(boundary);
boundary.faces = createBoxFaces(boundary.vertices);

% Assume the ground plane as the ring plane
LiDAR_opts.color_list = getColors(4);
LiDAR_opts.pose.centriod = [0 0 0];
LiDAR_opts.properties.beam = 32;




LiDAR_opts.properties.noise_sigma = opts.sensor_noise_level;

%% target orientation
% roll_s = 0;
% roll_e = 0;
% pitch_s = 0;
% pitch_e = 0;
% yaw_s = 0;
% yaw_e = 0;
% number_angles = 1;
% angle_list = [genRandomNumber(roll_s, roll_e, 1, number_angles), ...
%               genRandomNumber(pitch_s, pitch_e, 2, number_angles), ...
%               genRandomNumber(yaw_s, yaw_e, 3, number_angles)]; 
% x_s = 5;
% x_e = 5;
% y_s = 0;
% y_e = 0;
% z_s = 0;
% z_e = 0;
% translation_list = [genRandomNumber(x_s, x_e, 4, number_angles), ...
%                     genRandomNumber(y_s, y_e, 5, number_angles), ...
%                     genRandomNumber(z_s, z_e, 6, number_angles)]; 

if opts.use_best_shape && ~opts.data.simulation
    if opts.data.reload_matfiles
        % load everything in the folder with name containing '*Tag*mat'
        mat_files = loadMatFilesFromFolder(opts.path.mat_path, '*Target*.mat');
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
            data(t).mat_file = mat_files(t).name;
            data(t).point_cloud = getPayloadWithIntensity(pc(t).point_cloud, 1, opts.data.num_scans);
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
    vertices = load(opts.load_path + opts.filename);
    rotatated_ideal = [300 0 0];
    vertices.original_shape = moveByRPYXYZ(vertices.original_shape, rotatated_ideal, [0 0 0]);
    vertices.original_shape = vertices.original_shape(1:3, :);
    [angle_list, translation_list] = getAngleNTranslationList(opts.target_position_list);
    % positive x
%     angle_list = [20,  30, 30];
%     translation_list = [3.5, 0 0.1];
%     angle_list = [20,30,30];
%     translation_list = [3.5, 0 0.1];
%     translationT_list = [5, 5 0.5];
%     translation_list = [5, -5 0.5]; %

%     positive y
%     angle_list = [0,0,90];
%     translation_list = [0, 5 0.5]; 
%     translation_list = [-2, 5 0.5]; 
%     translation_list = [2, 5 0.5]; 

%     negative y
%     angle_list = [0,0,-90];
%     translation_list = [0, -5 0.5];
%     translation_list = [-2, -5 0.5];
%     translation_list = [2, -5 0.5];
% 
% %     negative x
%     angle_list = [0,0,180];
%     translation_list = [-5, 0 0.5];
%     translation_list = [-5, 2 0.5];
%     translation_list = [-5, -2 0.5];

    [object_list, color_list] = createOptimalShape(angle_list, translation_list, vertices.original_shape);
    txt = printStructure("", angle_list) + "-";
    txt = printStructure(txt, translation_list, 0);
    
    if LiDAR_opts.properties.sensor_noise_enable
        name = "noise_optimal_shape-" + txt;
%         save_fig_name = "noise1-neg_x3";
        save_fig_name = "noise1-pos_y3";
    else
        name = "optimal_shape-" + txt;
        save_fig_name = "ny1";
    end
    
    %% ideal target pose
    centroid = mean(vertices.original_shape, 2);
    ideal_translation_list = -centroid'; 
    offset = [0 0 0];
    % translation_list = [0 0 0.5]; 
    ideal_angle_list = [0, 0, 0]; 
    scale = 1;
    [ideal_object_list, ideal_color_list] = createOptimalShape(ideal_angle_list, ideal_translation_list - offset, scale*vertices.original_shape);
    [non_scale_ideal_object_list, ~] = createOptimalShape(ideal_angle_list, ideal_translation_list, vertices.original_shape);
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
    load(opts.load_path + name + ".mat")
    disp("Loaded LiDAR points on targets")
end

%%

project_name = "OptimalShape";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(8, 1, project_name, clear_fig, 1, 30);
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

pc_file = name + ".mat";
pc_file
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
original_pc = load(opts.load_path + pc_file); 
org_num_points = size(original_pc.object_list.points_mat, 2);

if org_num_points < 10
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

% % original loading
% if opts.use_edge_points
%     edge_points = load('edge_points.mat');
%     pc_points = reshape([edge_points.edge_points.Position].', 3,[]);
% else
%     pc_points = cur_pc.object_list.points_mat;
% end

% calculate the edge_points
pc_points = [];
for i = 1: size(original_pc.object_list.ring_points,2)
%%
if isequal(original_pc.object_list.ring_points(i).y, []) == 0
    line_size = size(original_pc.object_list.ring_points(i).y,2);
    [~, min_idx] = min(original_pc.object_list.ring_points(i).y);
    [~, max_idx] = max(original_pc.object_list.ring_points(i).y);
    tmp_points = [original_pc.object_list.ring_points(i).x(min_idx),
                  original_pc.object_list.ring_points(i).y(min_idx),
                  original_pc.object_list.ring_points(i).z(min_idx)];
    pc_points = [pc_points, tmp_points];
    if line_size ~= 1
        tmp_points = [original_pc.object_list.ring_points(i).x(max_idx),
                  original_pc.object_list.ring_points(i).y(max_idx),
                  original_pc.object_list.ring_points(i).z(max_idx)];
        pc_points = [pc_points, tmp_points];
    end
end
end

% testing only for showing the ring_points
% pc_points = [];
% count = 0;
% for i = 1: size(original_pc.object_list.ring_points,2)
% %%
% if isequal(original_pc.object_list.ring_points(i).y, []) == 0
%     count = count + 1;
%     line_size = size(original_pc.object_list.ring_points(i).y,2)
%     for j = 1:line_size
%         tmp_points = [original_pc.object_list.ring_points(i).x(j),
%                   original_pc.object_list.ring_points(i).y(j),
%                   original_pc.object_list.ring_points(i).z(j)];
%         pc_points = [pc_points, tmp_points];
%     end
% 
% end
% end
%
num_points = size(pc_points, 2);
fraction_left = num_points / org_num_points;


% transform current pc points
% fail
% 20, 30, 180/90/45/10/-150(non-tight)/-120(nt)/-110(nt);
% 180/90/45/30/25/22, 30, 0；
% 90/70/65, -30, 0；
% 

% succeed
% 20,30,0/-10/-20/-45/-90/-100;
% 60/50/45/20/-50/-90,-30,0;
% 
%%
rpy = [0,0,0];
% rp = genRandomNumber(-40,40,[],2);
% heading = genRandomNumber(-60,60,[],1);
% rpy = [rp(1), rp(2), heading];

% origin
% 10,5,1;
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



original_plane_correspondences = createOptimalShapePlaneCorrespondences(...
    original_pc.object_list.points_mat, ideal_object_list);
disp("formed original plane correspondences")

mode_list = [1,1,2,2,1,1,2,2,1,1];
mode_list = ones(1, opts.max_iter);

% H_final = eye(4);




% compute line equations
ideal_object_list = computeLineEquations(ideal_object_list);

solver.H_final = eye(4);
solver.H_previous = eye(4);
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
            compose("Initial Condition \nwith Subsampled PC"), [-90,0])
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
        cur_pc.object_list.centered_points(1, :), ...
        cur_pc.object_list.centered_points(2, :),  ...
        cur_pc.object_list.centered_points(3, :), ...
        'k.')
    scatter3(axes_h, ...
        transformed_pc(1, :), transformed_pc(2, :),  transformed_pc(3, :), ...
        'b.')
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    viewCurrentPlot(axes_h, "Results with Subsampled PC", [-90,0])

    
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
    viewCurrentPlot(axes_h, "Results with Full PC", [-90,0])
    
    
    [axes_h, fig_h] = getCurrentFigure(5, axes_handles, fig_handles);
    cla(axes_h)
    opts.simulate_lidar = 1;
    plotObjectsList(axes_h, opts, ...
        original_pc.LiDAR_opts, original_pc.object_list);
    scatter3(axes_h, ...
        estimated_vertices(1, :), ...
        estimated_vertices(2, :), ...
        estimated_vertices(3, :), 100, 'ro', 'fill')
    plot3(axes_h, ...
        [estimated_vertices(1, :), estimated_vertices(1, 1)], ...
        [estimated_vertices(2, :), estimated_vertices(2, 1)], ...
        [estimated_vertices(3, :), estimated_vertices(3, 1)], ...
        'r-', 'LineWidth', 3)
    viewCurrentPlot(axes_h, "Estimated Vertices with Full PC", [-90,0])
    
    

    if  norm(Log_SE3( affine \  solver.H_previous)) < 1e-5
        break;
    end
    cur_pc.object_list.points_mat = transformed_pc;
    solver.H_previous = affine;
    
    
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
disp("=================================")
disp("======= Estimated Results =======")
disp("=================================")
fprintf("Spent %.2f seconds\n", solver.computation_time)
RMS = sqrt(norm(...
    estimated_vertices - original_pc.object_list.object_vertices_mat_h, ...
    'fro').^2/4)
error_percentage = RMS / norm(object_list.centroid) * 100
geo_dis = norm(Log_SE3(solver.H_final/object_list.H_inv))
geo_rot = norm(Log_SE3(solver.H_final(1:3, 1:3)\object_list.H_inv(1:3,1:3)))
disp("ALL Done")
