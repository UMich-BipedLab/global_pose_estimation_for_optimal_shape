function [object_list, ideal_object_list, out_t] = loadTargetPose(opts, LiDAR_opts)

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
    out_t.name = [];
    
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
    
    [ideal_object_list, ~] = createOptimalShape(ideal_angle_list, ideal_translation_list - offset, vertices.original_shape);
    
elseif opts.use_best_shape && opts.data.simulation
    ideal_object_list = [];
    %% target pose
    vertices = load(opts.load_path + opts.filename);
    rotatated_ideal = [300 0 0];
    vertices.original_shape = moveByRPYXYZ(vertices.original_shape, rotatated_ideal, [0 0 0]);
    vertices.original_shape = vertices.original_shape(1:3, :);
    [out_t.angle_list, out_t.translation_list] = getAngleNTranslationList(opts.target_position_list);
    [object_list, ~] = createOptimalShape(out_t.angle_list, out_t.translation_list, vertices.original_shape);
    txt = printStructure("", out_t.angle_list) + "-";
    txt = printStructure(txt, out_t.translation_list, 0);
    
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
        out_t.name = "noise"+ num2str(noise_level) +"_optimal_shape-" + txt;
        %         save_fig_name = "noise1-neg_x3";
        save_fig_name = "noise"+num2str(noise_level)+"-pos_y3";
            
    else
        out_t.name = "optimal_shape-" + txt;
        save_fig_name = "ny1";
    end
    
elseif ~opts.use_best_shape && opts.data.simulation
    %% target pose
    out_t.angle_list = [30, 30, 20];
    out_t.translation_list = [5, 5 0.5];
%     translation_list = [-5, 5 0.5];
%     translation_list = [5, -5 0.5];
%     translation_list = [5, 5 -0.5];
%     translation_list = [-5, -5 0.5]; %%
%     translation_list = [-5, 5 -0.5];
%     translation_list = [5, -5 -50.5]; %%
%     translation_list = [-5, -5, -0.5];
%     translation_list = [5, 0, 0];
    
    
    opts_obs.target_size = 1;
    opts_obs.polygon = 4;
    opts_obs.rpy = out_t.angle_list;
    opts_obs.xyz = out_t.translation_list;
    [object_list, color_list] = createDynamicScene(opts_obs);
    txt = printStructure("", out_t.angle_list) + "-";
    txt = printStructure(txt, out_t.translation_list, 0);
    out_t.name = "squares_shape-" + txt;
    ideal_object_list = [];
end
end