function opts = loadDatasetInfo(opts)
    if opts.dataset == 0 
        opts.debug = 0;
        opts.verbose.output_level = 0;
        opts.verbose.selective = 0;
        opts.data.simulation = 1;
        opts.target_position_list = opts.target_num; % which angle/translation list to use


        if opts.target_position_list >= 100
            opts.sensor_noise_enable = 1;
            
            % nosie from spherical coordinates [Range, Az, El] <meter, deg, deg>
            if opts.target_position_list >= 100 && opts.target_position_list < 200 
                opts.sensor_noise_level = [0.0100, 0.1000, 0.1000];
            elseif opts.target_position_list >= 200 && opts.target_position_list < 300 
                opts.sensor_noise_level = [0.0300, 0.2000, 0.2000];
            elseif opts.target_position_list >= 300 && opts.target_position_list < 400 
                opts.sensor_noise_level = [0.03    0.3   0.3];
            elseif opts.target_position_list >= 400 && opts.target_position_list < 500 
                opts.sensor_noise_level = [0.05    0.5   0.5];
            else
                error("No such noise options: %i", opts.target_position_list)
            end
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
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/";
        opts.path.bagfile_root = "/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/";
        opts.path.event_name = "calibration";
        opts.data.simulation = 0;

         % user parameters
        opts.data.reload_matfiles = 0;
        opts.data.num_scans = 1;    
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;

    elseif opts.dataset == 5
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/";
        opts.path.event_name = "new_target_mat_files_08052020";
        opts.data.simulation = 0;

        % user parameters
        opts.data.reload_matfiles = 1;
        opts.data.num_scans = 1;
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;

    elseif opts.dataset == 6
        opts.path.bagfile_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/distance/bagfiles/";
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/";
        opts.path.event_name = "distance";
        opts.data.simulation = 0;

        % user parameters
        opts.data.reload_matfiles = 1;
        opts.data.num_scans = 1;
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;
    elseif opts.dataset == 7
        opts.path.bagfile_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/calibration/bagfiles/";
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-14-2021/";
        opts.path.event_name = "calibration";
        opts.data.simulation = 0;

        % user parameters
        opts.data.reload_matfiles = 1;
        opts.data.num_scans = 1;
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;
    elseif opts.dataset == 8
        opts.path.bagfile_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-18-2021/mocap/bagfile/";
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-18-2021/mocap/";
        opts.path.event_name = "matfiles";
        opts.data.simulation = 0;

        % user parameters
        opts.data.reload_matfiles = 1;
        opts.data.num_scans = 1;
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;
    elseif opts.dataset == 9
        opts.path.bagfile_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-22-2021/Bagfile/";
        opts.path.matfiles_root = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Jul-22-2021/";
        opts.path.event_name = "LiDARCleanMat";
        opts.data.simulation = 0;

        % user parameters
        opts.data.reload_matfiles = 1;
        opts.data.num_scans = 1;
        opts.data.num_rings = 32;
        opts.sensor_noise_enable = 0;
    else
        error("wrong dataset number: %i", opts.dataset)
    end

end