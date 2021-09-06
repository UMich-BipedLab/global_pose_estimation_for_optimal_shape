function [object_list, LiDAR_opts, boundary] = genLiDARPointsGivenObjectList(opts, object_list, LiDAR_opts, boundary)
    % Genereate uncalibrated LiDAR points
    if ~isfield(LiDAR_opts, 'properties') || ~isfield(LiDAR_opts, 'pose') || ~isfield(LiDAR_opts, 'mechanism')
        % LiDAR properties
        LiDAR_opts.properties.range = 150;
        LiDAR_opts.properties.return_once = 0;
        LiDAR_opts.pose.centriod = [0 0 0];
        LiDAR_opts.pose.rpy = [0 0 0]; % deg (roll pitch yaw)
        LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);
        LiDAR_opts.properties.mechanics_noise_model = 3; 
        LiDAR_opts.properties.sensor_noise_enable = 0;
        LiDAR_opts.properties.rpm = 1200; % 300, 600, 900, 1200
        LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
        [LiDAR_opts.properties.ring_elevation, ...
        LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties, 'ring_', LiDAR_opts.properties.beam);
        LiDAR_opts.mechanism.types = ["rotating-head", "solid-state"];
        LiDAR_opts.mechanism.type = 1;
    end

    if isempty(boundary)
        % Workspace boundary
        boundary.x = [160, -160];
        boundary.y = [160, -160];
        boundary.z = [160, -160];
        boundary.vertices = createBoxVertices(boundary);
        boundary.faces = createBoxFaces(boundary.vertices);
    end
    
    [object_list, ~, ~]= simulateLiDAR(object_list, boundary, LiDAR_opts);
end