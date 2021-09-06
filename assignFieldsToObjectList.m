function object_list = assignFieldsToObjectList(target_num, data)
    object_list.name = data(target_num).name;
    object_list.points_mat = data(target_num).point_cloud;
    object_list.target_scale = data(target_num).target_scale;
    
    % 1.643 is original size in meter
    object_list.target_size = 1.643 * object_list.target_scale; 
    [object_list.normal, object_list.centroid] = computePlane(object_list.points_mat(1:3, :));
end