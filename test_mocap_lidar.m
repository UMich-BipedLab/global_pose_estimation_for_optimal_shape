clear, clc
project_name = "CalMocapToLiDAR";
clear_fig = 1;
save_fig = 0;
fig = createFigureOptions(3, 1, project_name, clear_fig, 1, 30);
fig.save_figs = save_fig;
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);

%% user setting
save_results = 0;
range_num = 1;
[calibration_distance_range, validate_datasets] = loadCalibrationRange(range_num);


%% Load datasets
lidar_tables = load('table_cell-mocap-lidar.mat');
mocap_path = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-22-2021/MocapMat";



%% Check loaded datasets
num_lidar_data = size(lidar_tables.table_cell, 2);
mocap_mat_files = loadMatFilesFromFolder(mocap_path, '*.mat');
num_mocap_data = length(mocap_mat_files);

if num_mocap_data == 0
    error("No matfile is loaded, check data path: %s", mocap_path)
end
% for i = 1:num_mocap_data
%     fprintf("i: %2i; name: %s\n", i, mocap_mat_files(i).name)
% end
assert(num_lidar_data == num_mocap_data, ...
    "Num of datasets mismatch: LiDAR-%i, Mocap-%i", ...
    num_lidar_data, num_mocap_data);


%% load vertices to calibrates
lidar_vertices = [];
mocap_vertices = [];
for data_num = 1:num_lidar_data
    
    % skip angled datasets
    if ~mod(data_num, 2)
        continue
    end
    % LiDAR vertices
    est_results = lidar_tables.table_cell{data_num};
    est_name = lidar_tables.table_cell{data_num}.Name{1};
    template = lidar_tables.table_cell{data_num}.template{1};
    est_vertices = est_results.EstVertices{1}(1:3, :);
    est_centroid  = mean(est_vertices, 2);
    
    if est_centroid(1) < calibration_distance_range(1) || ...
       est_centroid(1) > calibration_distance_range(2)
        continue;
    end
    fprintf("Working on %s: %i/%i\n", est_name, data_num, num_lidar_data);
    

    
    % mocap vertices
    out = returnMocapVertices(mocap_mat_files, est_vertices, data_num);
    
    assert(all(out.mocap_name == est_name), ...
        "Dataset Mismatch: LiDAR-%s, Mocap-%s", est_name, out.mocap_name)
    
    lidar_vertices = [lidar_vertices, est_vertices(1:3, :)];
    mocap_vertices = [mocap_vertices, out.mocap_vertices(1:3, :)];
end


%% calibrate
s = 1.03; % 1.03;
mocap_vertices = s * mocap_vertices(1:3, :);
[fitting_error, vertex_mocap_to_lidar, H_ML] = ...
    computeProcrustes(mocap_vertices, lidar_vertices);
assert(fitting_error < 0.01, "gt is not correct: %.4f", fitting_error);
% save('H_ML_July-22-2021.mat', 'H_ML', 's', 'fitting_error')
calibration_RMSE = computeRMSE(lidar_vertices, vertex_mocap_to_lidar)
assert(calibration_RMSE < 0.04, "Calibration too bad: %.2f", calibration_RMSE);

%% plotting
[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);
cla(axes_h)
h1 = plotShape(axes_h, lidar_vertices, 'r', 'r');
h2 = plotShape(axes_h, vertex_mocap_to_lidar, 'b', 'b');
% plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
viewCurrentPlot(axes_h, "Calibration results", [-90, 0], 1)
% reloadCurrentPlot


%% check results one by one
% [axes_h, fig_h] = getCurrentFigure(2, axes_handles, fig_handles);
% for data_num = 1:size(lidar_vertices, 2)/4
%     cla(axes_h)
%     start_ind = 4 * (data_num - 1) + 1;
%     end_ind = start_ind + 3;
%     cur_lidar_vertices = lidar_vertices(:, start_ind : end_ind);
%     cur_mocap_vertices = vertex_mocap_to_lidar(:, start_ind : end_ind);
%     h1 = plotShape(axes_h, cur_lidar_vertices, 'r', 'r');
%     h2 = plotShape(axes_h, cur_mocap_vertices, 'b', 'b');
%     RMSE = computeRMSE(cur_lidar_vertices, cur_mocap_vertices)
%     
%     disp("Check results and press enter")
%     pause;
% end


%% lidar measurement
varNames = {'Name', 'Distance','QuantizationError','NumberOfPoints', ...
    'RMSE', 'RMSE_percentage', ...
    'translation_error', 'translation_error_percentage', 'rot_error', ...
    'EstVertices', 'EstH', 'GtVertices', 'GtH', 'points', 'template'};

result_cell = cell(1, length(validate_datasets));
for i = 1: length(validate_datasets)
    dataset = validate_datasets(i);
    est_results = lidar_tables.table_cell{dataset};
    est_name = lidar_tables.table_cell{dataset}.Name{1};
    template = lidar_tables.table_cell{dataset}.template{1};
    est_vertices = est_results.EstVertices{1}(1:3, :);
    est_H = est_results.EstH{1};
    lidar_points = est_results.points{1};
    target_centroid = mean(lidar_points, 2);


    % compute gt data
    out = returnMocapVertices(mocap_mat_files, est_vertices, dataset);
    M_mocap_vertices = s * out.mocap_vertices;
    L_mocap = H_ML * convertToHomogeneousCoord(M_mocap_vertices);

    [gt_error, ~, gt_H] = ...
        computeProcrustes(template(1:3, :), L_mocap(1:3, :));
    assert(gt_error < 0.01, "gt not accurate: %.2f", gt_error)



    % results
    disp("=================================")
    disp("Results")
    disp("=================================")
    format short
    fprintf("LiDAR file: %s\n", est_name)
    fprintf("Mocap file: %s\n", out.mocap_name)
    target_distance = norm(target_centroid(1:3))
    RMSE = computeRMSE(est_vertices, L_mocap(1:3, :))
    geo_rot = rad2deg(norm(Log_SO3(gt_H(1:3, 1:3)/est_H(1:3, 1:3))))
    trans_error = norm(gt_H(1:3, 4) - est_H(1:3, 4))
    
    
    trans_error_percentage = trans_error / target_distance * 100;
    RMSE_percentage = RMSE / target_distance * 100;
    
    quantization_error = deg2rad(0.4) * target_distance;
    num_point_on_target = size(lidar_points, 2);
    
    
    % put results to table
    digit = 3;
    T = table({est_name}, target_distance, ...
        round(quantization_error, digit), int32(num_point_on_target), ...
    round(RMSE, digit), round(RMSE_percentage, digit), ...
    round(trans_error, digit), round(trans_error_percentage, digit), ...
    round(geo_rot, digit), ...
    {est_vertices}, {est_H}, {L_mocap}, {gt_H}, {lidar_points}, ...
    {template}, 'VariableNames', varNames);

    result_cell{i} = T;

    [axes_h, fig_h] = getCurrentFigure(3, axes_handles, fig_handles);
    cla(axes_h)
    scatter3(axes_h, lidar_points(1, :), lidar_points(2, :), lidar_points(3, :), 'c.')
    h1 = plotShape(axes_h, est_vertices, 'r', 'r');
    h2 = plotShape(axes_h, L_mocap, 'b', 'b');
    % plotOriginalAxisWithText(axes_h, "Origin", eye(4), 0.5)
    % viewCurrentPlot(axes_h, "", [-50, 30])
    viewCurrentPlot(axes_h, "Results-" + est_name, [-90, 0], 0)
end

if save_results
    save("exp_result-" + num2str(range_num) + ".mat", 'result_cell');
end






























