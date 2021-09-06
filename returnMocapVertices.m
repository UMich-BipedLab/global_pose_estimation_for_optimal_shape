function out = returnMocapVertices(mocap_mat_files, lidar_vertices, data_num)
    % mocap data
    mocap_name = mocap_mat_files(data_num).name;
    mocap_name = mocap_name(1:strfind(mocap_name, '.')-1);

    
    
    
    mocap_to_load = mocap_mat_files(data_num).file_name;
    mocap_file = load(mocap_to_load);
    field_names = fieldnames(mocap_file);
    mocap_tarj = mocap_file.(field_names{1}).Trajectories;
    mocap_rigid_body = mocap_file.(field_names{1}).RigidBodies;

    % find target indices
    lidar_vertex_ind = find(contains(mocap_tarj.Labeled.Labels, 'Target'));
    
    % find a scan with no nan value
    num_mocap_scan = size(mocap_tarj.Labeled.Data, 3);
    is_nan = true;
    for i = 1:num_mocap_scan
        mocap_lidar_vertices = squeeze(mocap_tarj.Labeled.Data(lidar_vertex_ind, 1:3 , i))' ./ 1000;
        num_nan = sum(sum(isnan(mocap_lidar_vertices)));
        if num_nan == 0
            break;
            is_nan = false;
        end
    end

    assert(is_nan, "Mocap data has nan")

    num_vertices = size(mocap_lidar_vertices, 2);
    assert(num_vertices == 4, "wrong number of vertices: %i", num_vertices)
    
    v = 1:4;
    possible_orders = perms(v);

    d_final = 1e5;
    order_final = [];
    mocap_vertices_final = [];
    for i = 1:size(possible_orders, 1)
        cur_order = possible_orders(i, :);
        mocap_lidar_vertices = mocap_lidar_vertices(1:3, cur_order); 
        [cur_d, ~, ~] = computeProcrustes(mocap_lidar_vertices(1:3, :), lidar_vertices(1:3, :));
        if cur_d < d_final
            d_final = cur_d;
            order_final = cur_order;
            mocap_vertices_final = mocap_lidar_vertices;
        end
    end
    out.mocap_name = mocap_name;
    out.d_final = d_final;
    out.order_final = order_final;
    out.mocap_vertices = mocap_vertices_final;
end