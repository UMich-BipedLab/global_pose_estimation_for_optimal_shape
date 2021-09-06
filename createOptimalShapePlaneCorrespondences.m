function correspondences = createOptimalShapePlaneCorrespondences(...
    points, planes)
    assert(length(points) >= 5, "not enough points")
    points = makeWideMatrix(points);
    correspondences = [];
    if ~isfield(planes, 'unit_normal')
        planes.unit_normal = planes.normal ./ norm(planes.normal);
    end
    plane = Plane(planes.centroid(1:3), planes.unit_normal);
%     plane = Plane([4 0 0]', planes.unit_normal);
    for j = 1: size(points, 2)
%         point = plane.project(Point(points(1:3,j)));
%         point.x;
        point = Point(points(1:3,j));
        correspondences = [correspondences, Point2Plane(point, plane)];
    end
end