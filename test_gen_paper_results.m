% This script does the following


clc, clear
run('/home/brucebot/workspace/test/CVPR17/setup.m')
addpath(genpath('/home/brucebot/workspace/lc-calibration/L1_relaxation'));
poolobj = gcp('nocreate');
rmpath(genpath('/home/brucebot/workspace/lc-calibration/LiDAR_intrinsic_calibration'))
% If no pool, do not create new one.
if isempty(poolobj)
    parpool('eight-threading');
end

project_name = "OptimalShape";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(10, 1, project_name, clear_fig, 1, 30);
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);
disp("figure handles created")

% paths 
opts.save_path = "./data/July-2021/";
opts.load_path = "./data/July-2021/";
opts.filename = "original_shape.mat";


% user parameters
opts.regen_LiDAR_points = 0; % 0: if file exist, use it, or regenerate it
opts.use_best_shape = 1;
opts.plot = 1;
opts.save_plots = 0;

% 0: simulation
% 1: lab 
% 2: wavefield
% 3: robotics building
% 4: calibration (25 targets):  BAD opts.target_num: 16, 21
% 5: old data (no images, 18 targets): GOOD opts.target_num = 2, 6, 7, 8, 9, 11, 12
% 6: vary distance 7/14/2021 (max 15)
% 7: calibration data 7/14/2021 (max 12)
opts.dataset = 0; 

% In exp: which target to use. 
% In sim, which list of angle and translation to use 
% check getAngleNTranslationList for reference
%  1-99: no noise
% 101-199, level 1 noise
% 201-299, level 2 noise
% 301-399, level 3 noise
% 401-499, level 4 noise
opts.target_num = 1;




opts = loadDatasetInfo(opts);



if opts.dataset~=0
    opts.save_path = "./paper_results/experiments/";
    opts.path.mat_path = opts.path.matfiles_root + opts.path.event_name + "/";
end
checkDirectory(opts.save_path)


%% LiDAR properties
[LiDAR_opts, boundary] = loadLiDARProperties(opts);


%% target orientation
[object_list, ideal_object_list, target_properties] = loadTargetPose(opts, LiDAR_opts);



%% LiDAR Simulation
if opts.regen_LiDAR_points && opts.data.simulation
    [object_list, LiDAR_opts, boundary] = genLiDARPointsGivenObjectList(opts, object_list, LiDAR_opts, boundary);
    save(opts.save_path + target_properties.name + ".mat", 'object_list', 'LiDAR_opts', 'boundary', 'color_list')
    disp("Generated LiDAR points on targets")
elseif ~opts.regen_LiDAR_points && opts.data.simulation
    file_to_be_load = opts.load_path + target_properties.name + ".mat";
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
    pc_file = target_properties.name + ".mat";
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
disp("LiDAR pc loaded")
%% calculate the edge points
if opts.data.simulation
    edge_points = findEdgePointsFromLiDARPoints(original_pc);
else
    edge_points = [];
    for ring = 1: opts.data.num_rings
        ring_points = original_pc.object_list.points_mat(:, original_pc.object_list.points_mat(5,:)==ring);
        num_ring_points = size(ring_points, 2);
        if num_ring_points > 0
            edge_points = [edge_points, ring_points(1:3, 1)];
            if num_ring_points > 1
               edge_points = [edge_points, ring_points(1:3, end)];
            end
        end
        
    end
end


% rpy = [0,0,0];
% 
% xyz = [0,0,0];
% if opts.data.simulation
%     [cur_pc.object_list, ~] = createOptimalShape(...
%         rpy, xyz, convertXYZstructToXYZmatrix(...
%         cur_pc.object_list.object_vertices));
% end
% 
% 
% H_trans = constructHByRPYXYZMovingPoints(rpy, xyz);
% cur_pc.object_list.points_mat = H_trans * ...
%         convertToHomogeneousCoord(edge_points(1:3, :));
% initial_pc =   cur_pc.object_list.points_mat; 
% disp("pc transformed")    


original_pc_points = original_pc.object_list.points_mat;
% if opts.data.simulation
%     [original_pc.object_list, ~] = createOptimalShape(...
%         rpy, xyz, convertXYZstructToXYZmatrix(...
%         original_pc.object_list.object_vertices));
% end
% original_pc.object_list.points_mat = H_trans * ...
%         convertToHomogeneousCoord(original_pc_points(1:3, :));
    
% disp("pc transformed")     



original_plane_correspondences = createOptimalShapePlaneCorrespondences(...
    original_pc.object_list.points_mat, ideal_object_list);
disp("formed original plane correspondences")




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

out_t = estimateLiDARPose(opts, cur_pc, ideal_object_list);
estimated_vertices = out_t.estimated_vertices;
solver = out_t.solver;

%%
disp("=================================")
disp("======= Estimated Results =======")
disp("=================================")
fprintf("Spent %.2f seconds\n", solver.computation_time)
if opts.data.simulation
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
    
    
    
    fprintf("Geodesic Distance on SE(3) (norm(Log(H_gt * inv(H_est)))): %.4f \n", geo_dis)
    fprintf("Geodesic Distance on SO(3) (norm(Log(R_gt * inv(R_est)))): %.4f [deg]\n", geo_rot)
    fprintf("Translation Error: %.4f [m]\n", trans_error)
    fprintf("Percentage of Translation Error: %.4f [%%] \n", trans_error_percentage)
    fprintf("Percentage of Vertices Error: %.4f [%%] \n", RMSE_percentage)
    fprintf("RMSE: %.2f [m]\n", RMSE)
else
    fprintf("Target Distance: %.2f [m]\n",  norm(object_list.centroid))
    fprintf("original_cost: %.2f [m]\n", sum(abs(original_plane_correspondences.cost(Pose()))))
    fprintf("optimized_cost: %.2f [m]\n", solver.optimized_cost)
end
%%
if opts.data.simulation
if ~isfile('results.csv')
    fid = fopen('results.csv', 'A' );
    fprintf(fid, 'x, y, z, r, p, h, rmse, rmse_per, geo_dis, geo_rot, trans_error, trans_error_per\n');
else
    fid = fopen('results.csv', 'A' );
end
fprintf(fid, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', ...
    target_properties.translation_list(1), target_properties.translation_list(2), target_properties.translation_list(3), ...
    target_properties.angle_list(1), target_properties.angle_list(2), target_properties.angle_list(3), ...
    RMSE, RMSE_percentage, geo_dis, geo_rot, ...
    trans_error, trans_error_percentage);
fclose( fid );
end
disp("=================================")
disp("ALL Done")
