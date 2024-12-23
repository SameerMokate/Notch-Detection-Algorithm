function threePointPlanes2
    % Load the 3D model
    model = stlread('D:\Work\VAKA - Work\WeldScanAlgorithm\butt_weld_cropped.stl');

    % Display the 3D geometry
    figure;
    trisurf(model.ConnectivityList, model.Points(:,1), model.Points(:,2), model.Points(:,3), ...
        'FaceColor', 'yellow', 'EdgeColor', 'none');
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Select 3 Points to Define Planes');
    view(3);
    camlight;
    lighting gouraud;
    hold on;

    % Prompt user for number of planes
    numPlanes = input('Enter the number of planes to create (1, 2, or 3): ');
    
    % Check for valid input
    if ~ismember(numPlanes, [1, 2, 3])
        error('Invalid input. Please enter 1, 2, or 3 for the number of planes.');
    end

    % Loop for the specified number of planes
    for planeNum = 1:numPlanes
        disp(['Select 3 points for Plane ', num2str(planeNum)]);
        selectedPoints = select3Points(model.Points); % Select 3 points
        plot3(selectedPoints(:,1), selectedPoints(:,2), selectedPoints(:,3), 'ro', ...
              'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Visualize selected points
        createPlaneFromPoints(selectedPoints, planeNum); % Create and plot the plane
    end
end

% Function to select 3 points
function selectedPoints = select3Points(points)
    selectedPoints = zeros(3, 3); % Initialize 3x3 array for storing selected points
    for i = 1:3
        [x, y] = ginput(1); % Get user-selected x and y
        z = getZFromXY(points, x, y); % Calculate closest z from geometry
        selectedPoints(i, :) = [x, y, z];
        disp(['Point ', num2str(i), ': [', num2str(x), ', ', num2str(y), ', ', num2str(z), ']']);
    end
end

% Function to find z-coordinate closest to the selected x, y
function z = getZFromXY(points, x, y)
    distances = sqrt((points(:,1) - x).^2 + (points(:,2) - y).^2); % Compute distances
    [~, idx] = min(distances); % Find the closest point
    z = points(idx, 3); % Return the z-coordinate of the closest point
end

% Function to create and display a plane from 3 points
function createPlaneFromPoints(points, planeNumber)
    % Validate input
    if size(points, 1) ~= 3
        error('Exactly 3 points are required to define a plane.');
    end

    % Define the plane using 3 points
    P1 = points(1, :);
    P2 = points(2, :);
    P3 = points(3, :);

    % Calculate plane normal
    v1 = P2 - P1;
    v2 = P3 - P1;
    normal = cross(v1, v2);
    normal = normal / norm(normal); % Normalize the normal vector

    % Plane equation: Ax + By + Cz + D = 0
    D = -dot(normal, P1);

    % Create a grid for the plane (adjusted dynamically based on geometry size)
    padding = 5; % Padding around selected points
    xRange = max(points(:,1)) - min(points(:,1));
    yRange = max(points(:,2)) - min(points(:,2));
    gridResolution = max(30, ceil(max(xRange, yRange))); % Dynamically adjust resolution

    xMin = min(points(:,1)) - padding;
    xMax = max(points(:,1)) + padding;
    yMin = min(points(:,2)) - padding;
    yMax = max(points(:,2)) + padding;

    [x, y] = meshgrid(linspace(xMin, xMax, gridResolution), linspace(yMin, yMax, gridResolution));
    z = (-normal(1) * x - normal(2) * y - D) / normal(3);

    % Display the plane
    surf(x, y, z, 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'FaceColor', rand(1, 3));
    disp(['Plane ', num2str(planeNumber), ' Equation: ', ...
          num2str(normal(1)), 'x + ', num2str(normal(2)), 'y + ', ...
          num2str(normal(3)), 'z + ', num2str(D), ' = 0']);
end
