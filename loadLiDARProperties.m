function [LiDAR_opts, boundary] = loadLiDARProperties(opts)
    % LiDAR properties
    LiDAR_opts.properties.range = 150;
    LiDAR_opts.properties.return_once = 0;
    LiDAR_opts.pose.centriod = [0 0 0];
    LiDAR_opts.pose.rpy = [180 0 0]; % deg (roll pitch yaw)
    LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);
    LiDAR_opts.properties.mechanics_noise_model = 0; 
    LiDAR_opts.properties.sensor_noise_enable = opts.sensor_noise_enable;
    LiDAR_opts.properties.rpm = 1200; % 300, 600, 900, 1200
    if isfield(opts, 'sensor_noise_level')
        LiDAR_opts.properties.noise_sigma = opts.sensor_noise_level;
    end
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
        
end