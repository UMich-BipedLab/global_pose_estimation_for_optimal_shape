function [calibration_distance_range, validate_datasets] = loadCalibrationRange(range_num)
    switch range_num
        case 1
            calibration_distance_range = [0, 5];
            validate_datasets = 1:4;
        case 2
            calibration_distance_range = [5, 10];
            validate_datasets = 5:8;
        case 3
            calibration_distance_range = [10, 13];
            validate_datasets = 9:12;
        case 4
            calibration_distance_range = [13, 16];
            validate_datasets = 13:16;
        case 5
            calibration_distance_range = [15, 19];
            validate_datasets = 17:18;
        otherwise
    end
end