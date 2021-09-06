function out_t = downsampleToPercentage(in_t, percentage)
    num_points = size(in_t.points_mat, 2);
    in_t.ring_points = [];
    in_t.noise_less_ring_points = [];
    indices = rand(1, num_points);
    keep_indices = indices < percentage;
%     sum(keep_indices)/num_points
    in_t.points_mat = in_t.points_mat(:, keep_indices);
    out_t = in_t;
end