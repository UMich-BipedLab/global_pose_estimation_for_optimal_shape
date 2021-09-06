function [edge_points, target_points] = findEdgePointsFromLiDARPoints(opts, original_pc)
    % calculate the edge_points
    edge_points = [];
    target_points = [];
    for ring = 1: size(original_pc.object_list.ring_points, 2)
        if opts.target_occlusion && ismember(ring, opts.remove_rings)
            continue
        end
%         ring;
%         num_ring_points = size(original_pc.object_list.ring_points(ring).x, 2);
%         if num_ring_points == 0
%             continue
%         elseif num_ring_points >= 2
%             p1 = [original_pc.object_list.ring_points(ring).x(1); ...
%                 original_pc.object_list.ring_points(ring).y(1);...
%                 original_pc.object_list.ring_points(ring).z(1)];
%             p2 = [original_pc.object_list.ring_points(ring).x(end); ...
%                 original_pc.object_list.ring_points(ring).y(end);...
%                 original_pc.object_list.ring_points(ring).z(end)];
%             edge_points = [edge_points, p1, p2];
%         elseif num_ring_points == 1
%             p1 = [original_pc.object_list.ring_points(ring).x(1); ...
%                 original_pc.object_list.ring_points(ring).y(1);...
%                 original_pc.object_list.ring_points(ring).z(1)];
%             edge_points = [edge_points, p1];
%         end
        xs = original_pc.object_list.ring_points(ring).x;
        ys = original_pc.object_list.ring_points(ring).y;
        zs = original_pc.object_list.ring_points(ring).z;
        points = [xs; ys; zs];
        target_points = [target_points, points];
        if isequal(original_pc.object_list.ring_points(ring).y, []) == 0
            line_size = size(original_pc.object_list.ring_points(ring).y,2);
            [~, min_idx] = min(original_pc.object_list.ring_points(ring).y);
            [~, max_idx] = max(original_pc.object_list.ring_points(ring).y);
            tmp_points = [original_pc.object_list.ring_points(ring).x(min_idx),
                original_pc.object_list.ring_points(ring).y(min_idx),
                original_pc.object_list.ring_points(ring).z(min_idx)];
            edge_points = [edge_points, tmp_points];
            if line_size ~= 1
                tmp_points = [original_pc.object_list.ring_points(ring).x(max_idx),
                    original_pc.object_list.ring_points(ring).y(max_idx),
                    original_pc.object_list.ring_points(ring).z(max_idx)];
                edge_points = [edge_points, tmp_points];
            end
        end
    end
end