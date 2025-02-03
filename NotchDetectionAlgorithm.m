clearvars;
close all;
clc;

%% Load the Model
model = stlread('D:\Work\VAKA - Work\Notch Detection Algorithm\butt_weld_cropped1.stl');
% Extracts X,Y,Z points and creates a point cloud.
X = model.Points(:, 1);
Y = model.Points(:, 2);
Z = model.Points(:, 3);
ptCloud = pointCloud([X, Y, Z]);

% First Display
figure;
trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
title('3D Model');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
camlight;
lighting gouraud;

%% Grid Generation
gridResolution = 25; % Change -- Number of grid points on each axis.

% This finds a rectangular box around the point cloud by determining min
% and max of X and Y direction to ensure the grid does not go outside the
% geometry or the point cloud
xMin = min(ptCloud.Location(:,1));
xMax = max(ptCloud.Location(:,1));
yMin = min(ptCloud.Location(:,2));
yMax = max(ptCloud.Location(:,2));

% This generates evenly spaced grid between min and max.
[x, y] = meshgrid(linspace(xMin, xMax, gridResolution), ...
                  linspace(yMin, yMax, gridResolution));

F = scatteredInterpolant(X, Y, Z, 'natural', 'none'); % Interpolation function which estimates z values based on X and Y.
z = F(x, y); % Evaluates interpolation function at each grid point.

gridPoints = [x(:), y(:), z(:)];  % Stored grid points in array.

% Second (Mesh grid) display starts
figure;
pcshow(ptCloud);
hold on;

for i = 1:gridResolution  % This loop draws mesh lines using the grid points above
    plot3(x(:, i), y(:, i), z(:, i), 'k-', 'LineWidth', 1.5);
    plot3(x(i, :), y(i, :), z(i, :), 'k-', 'LineWidth', 1.5);
end

plot3(gridPoints(:,1), gridPoints(:,2), gridPoints(:,3), 'r.', 'MarkerSize', 10);
title('3D Grid Over Weld Seam Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Sphere Placement
sphereRadius = 1; % Change -- Sphere radius for initial weld seam and flat surface detection.
verticalTolerance = 0.1; % Change -- Tolerance for the verticle points to determine if it is weld seam or just a small bump.

% Thrid Display -- Black and Magenta spheres starts.
figureHandle = figure;
pcshow(ptCloud);
hold on;

% Preallocates an array of graphics objects with the same size as the grid (x).
sphereHandles = gobjects(size(x)); % Which allows later modifications and tracking of the spheres.
magentaCenters = [];  % Since weld seam detection is done with centre of the spheres, this stores those magenta sphere centres.

for i = 1:size(x, 1)  % This loop lays the spheres on the grid.
    for j = 1:size(x, 2)
        center = [x(i, j), y(i, j), z(i, j)]; % Gets grid points from above as sphere centres for start.
        distances = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2); % Computes distances to all points in point cloud.
        
        % This checks the height for weld seam detection inside a sphere.
        verticalSeamInteraction = any(distances < sphereRadius & abs(Z - center(3)) > verticalTolerance);
        
        if verticalSeamInteraction
            sphereColor = 'magenta';
            magentaCenters = [magentaCenters; center];
        else
            sphereColor = 'black';
        end
        
        % This creates spheres and place it on grid points
        [sx, sy, sz] = sphere;
        sx = sx * sphereRadius + center(1);
        sy = sy * sphereRadius + center(2);
        sz = sz * sphereRadius + center(3);
        h = surf(sx, sy, sz, 'FaceColor', sphereColor, 'EdgeColor', 'none', ...
                 'ButtonDownFcn', @(src, event) toggleSphereColor(src), ...
                 'HitTest', 'on', 'PickableParts', 'all');
        sphereHandles(i, j) = h;
    end
end

title('Grid Points with Spheres for Flat Surface Detection');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off;

%% Manual Selection
uicontrol('Style', 'pushbutton', 'String', 'Continue', ...
          'Position', [20, 20, 100, 40], 'Callback', @(~,~) uiresume(figureHandle));

uiwait(figureHandle);

% It's in the name
function toggleSphereColor(sphere)
    currentColor = sphere.FaceColor;
    if isequal(currentColor, [1, 0, 1])
        sphere.FaceColor = 'black';
    else
        sphere.FaceColor = [1, 0, 1];
    end
end

%% Flat Surface Removal
tolerance = sphereRadius; % Defines tolerance as sphere radius.

magentaCenters = []; % Initialises an empty array to store magenta sphere centres.

% This loops through all the spheres to find magenta ones.
for i = 1:size(sphereHandles, 1)
    for j = 1:size(sphereHandles, 2)
        sphere = sphereHandles(i, j);
        if isvalid(sphere) && isequal(sphere.FaceColor, [1, 0, 1]) % Checks for valid and magenta sphere.

            % Computes the centre of the magenta sphere by averaging its X, Y, Z values.
            center = [mean(sphere.XData(:)), mean(sphere.YData(:)), mean(sphere.ZData(:))];

            % Stores centres of magenta sphere for later use.
            magentaCenters = [magentaCenters; center];
        end
    end
end

% If no spheres are selected then this warning but proceeds anyway to
% further code.
if isempty(magentaCenters)
    error('No magenta spheres selected. Ensure spheres are manually selected before continuing.');
end

% Extracts only X and Y coordinates to defind the boundary to filter out
% flat surface.
magentaX = magentaCenters(:, 1);
magentaY = magentaCenters(:, 2);

%% Flat Surface Filtering
yValues = unique(magentaY); % Gets unique Y-values of magenta spheres.
xBorderMin = zeros(size(yValues)); % Initialises min X boundary for each Y-level
xBorderMax = zeros(size(yValues)); % Initialises max X boundary for each Y-level

% This loops through to get X boundary for each Y-level.
for i = 1:length(yValues)
    currentY = yValues(i); % Gets current Y-level.
    indices = abs(magentaCenters(:, 2) - currentY) < tolerance; % Checks for sphere within tolerance.

    if any(indices)
        % Finds min and max X-values for current Y-level.
        xBorderMin(i) = min(magentaCenters(indices, 1));
        xBorderMax(i) = max(magentaCenters(indices, 1));
    else
        if i > 1 % If no magenta sphere at this level, then takes the value from last row.
            xBorderMin(i) = xBorderMin(i-1);
            xBorderMax(i) = xBorderMax(i-1);
        end
    end
end

% This steps just gives extra margin at the boundary to smooth out the
% filtering.
xBorderMin = xBorderMin - tolerance;
xBorderMax = xBorderMax + tolerance;

% This interpolates xBorderMin and xBorderMax across all Y-values in the point cloud and uses linear interpolation to smooth out the boundaries.
% The extrap (extrapolation) ensures all Y-values in the cloud are covered.
interpXMin = interp1(yValues, xBorderMin, ptCloud.Location(:, 2), 'linear', 'extrap');
interpXMax = interp1(yValues, xBorderMax, ptCloud.Location(:, 2), 'linear', 'extrap');

% This keeps points within specified range and removes the rest.
validIndices = ...
    (ptCloud.Location(:, 1) >= interpXMin & ptCloud.Location(:, 1) <= interpXMax) & ...
    (ptCloud.Location(:, 2) >= min(yValues) - tolerance & ...
     ptCloud.Location(:, 2) <= max(yValues) + tolerance);

filteredPoints = ptCloud.Location(validIndices, :); % This extracts the valid points.

ptCloudFiltered = pointCloud(filteredPoints); % This stores those valid points.

% Fourth display to display the filter point clound (Weld seam region)
figure;
pcshow(ptCloudFiltered);
hold on;

%% Smallest Notch Detection
numMeshSteps = 7; % Change
meshRefinementFactor = 2; % Change
minSphereRadius = 0.2; % Change
maxSphereRadius = 5.0; % Change
numSmallestSpheres = 100; % Change

% Takes values from filtered point cloud.
X = ptCloudFiltered.Location(:, 1);
Y = ptCloudFiltered.Location(:, 2);
Z = ptCloudFiltered.Location(:, 3);

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');

% Initialisations
smallestSphereRadius = inf; % This will later stores smallest sphere radius found in each step.
smallestSphereCenter = [];
smallestSphereStep = -1;
smallestSpheres = [];

% This loops through mesh refinement steps
for step = 1:numMeshSteps
    gridResolution = 25 + step * meshRefinementFactor; % Change -- increases mesh resolution

    % Same as above. Generates equally distant grid.
    [xMesh, yMesh] = meshgrid(linspace(min(X), max(X), gridResolution), ...
                              linspace(min(Y), max(Y), gridResolution));
    zMesh = F(xMesh, yMesh); % Interpolate Z values.
    
    % This loops through each mesh point to check if the sphere fits.
    for i = 1:size(xMesh, 1)
        for j = 1:size(xMesh, 2)
            % Defines current mesh point as potential sphere centre.
            center = [xMesh(i, j), yMesh(i, j), zMesh(i, j)];
            distances = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2); % Calculates the distance of the current point from other points in point cloud.
            
            % Filters out distances that are outside the sphere radius
            % range and stores the ones which satisfy this condition as
            % potential candidates.
            radiusCandidates = distances(distances > minSphereRadius & distances < maxSphereRadius);

            % Change -- at least 3 points must touch the sphere bottom
            % surface.
            if length(radiusCandidates) < 50, continue; end      %% Changes required %%

            validRadius = min(radiusCandidates); % This finds the smallest sphere from the candidates.

             % This stores smallest sphere found in each mesh step and
             % updates it if it finds smaller than the current sphere in upcoming steps.
            if validRadius < smallestSphereRadius

                % Stores the values at each steps and update them if
                % smaller than current sphere is found in next step until
                % the end step. If smaller than current sphere is not
                % found, then these keep the value from last step.
                smallestSphereRadius = validRadius;
                smallestSphereCenter = center;
                smallestSphereStep = step;
            end

            % This stores the candidate sphere if it's within the smallest 100.
            smallestSpheres = [smallestSpheres; struct('radius', validRadius, 'center', center)];
            
            % This keeps only the smallest 100 by sorting and trimming out
            % all the saved candidates
            if length(smallestSpheres) > numSmallestSpheres
                [~, sortIdx] = sort([smallestSpheres.radius]);
                smallestSpheres = smallestSpheres(sortIdx(1:numSmallestSpheres));
            end
        end
    end
end

% Command line output.
disp(['Smallest sphere radius: ', num2str(smallestSphereRadius)]);
disp(['Smallest sphere center: [', num2str(smallestSphereCenter), ']']);
disp(['Found at mesh step: ', num2str(smallestSphereStep)]);

%% Displaying Smallest Notch (The smallest sphere is displayed by red colour and white arrow and set of smallest spheres are shown by white colour.)
if ~isempty(smallestSphereCenter)
    % Adds a large red arrow pointing to the smallest sphere centre
    quiver3(smallestSphereCenter(1), smallestSphereCenter(2), smallestSphereCenter(3), ...
            0, 0, 10, 'Color', 'white', 'LineWidth', 3, 'MaxHeadSize', 5);
    % Adds a red sphere at the smallest sphere centre
    scatter3(smallestSphereCenter(1), smallestSphereCenter(2), smallestSphereCenter(3), ...
                 50, 'r', 'filled');

    % This plots all 'numSmallestSpheres' in the same display.
    for k = 1:length(smallestSpheres)
    scatter3(smallestSpheres(k).center(1), ...
             smallestSpheres(k).center(2), ...
             smallestSpheres(k).center(3), ...
             25, 'w', 'filled'); % White markers for all small spheres
    end
else
    warning('No valid sphere found for visualization.');
end

hold off;
