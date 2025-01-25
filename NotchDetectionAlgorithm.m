clearvars;
close all;
clc;

%% Load the Model
model = stlread('D:\Work\VAKA - Work\WeldScanAlgorithm\butt_weld_cropped.stl');
X = model.Points(:, 1);
Y = model.Points(:, 2);
Z = model.Points(:, 3);
ptCloud = pointCloud([X, Y, Z]);

figure;
trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
title('3D Model');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
camlight;
lighting gouraud;

%% Grid Generation
gridResolution = 25; % Change
xMin = min(ptCloud.Location(:,1));
xMax = max(ptCloud.Location(:,1));
yMin = min(ptCloud.Location(:,2));
yMax = max(ptCloud.Location(:,2));

[x, y] = meshgrid(linspace(xMin, xMax, gridResolution), ...
                  linspace(yMin, yMax, gridResolution));

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');
z = F(x, y);

gridPoints = [x(:), y(:), z(:)];

figure;
pcshow(ptCloud);
hold on;

for i = 1:gridResolution
    plot3(x(:, i), y(:, i), z(:, i), 'k-', 'LineWidth', 1.5);
    plot3(x(i, :), y(i, :), z(i, :), 'k-', 'LineWidth', 1.5);
end

plot3(gridPoints(:,1), gridPoints(:,2), gridPoints(:,3), 'r.', 'MarkerSize', 10);
title('3D Grid Over Weld Seam Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Sphere Placement
sphereRadius = 1; % Change
verticalTolerance = 0.1; % Change

figureHandle = figure;
pcshow(ptCloud);
hold on;

sphereHandles = gobjects(size(x));
magentaCenters = [];

for i = 1:size(x, 1)
    for j = 1:size(x, 2)
        center = [x(i, j), y(i, j), z(i, j)];
        distances = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2);
        
        verticalSeamInteraction = any(distances < sphereRadius & abs(Z - center(3)) > verticalTolerance);
        
        if verticalSeamInteraction
            sphereColor = 'magenta';
            magentaCenters = [magentaCenters; center];
        else
            sphereColor = 'black';
        end

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

function toggleSphereColor(sphere)
    currentColor = sphere.FaceColor;
    if isequal(currentColor, [1, 0, 1])
        sphere.FaceColor = 'black';
    else
        sphere.FaceColor = [1, 0, 1];
    end
end

%% Flat Surface Removal
tolerance = sphereRadius;

magentaCenters = [];
for i = 1:size(sphereHandles, 1)
    for j = 1:size(sphereHandles, 2)
        sphere = sphereHandles(i, j);
        if isvalid(sphere) && isequal(sphere.FaceColor, [1, 0, 1])
            center = [mean(sphere.XData(:)), mean(sphere.YData(:)), mean(sphere.ZData(:))];
            magentaCenters = [magentaCenters; center];
        end
    end
end

if isempty(magentaCenters)
    error('No magenta spheres selected. Ensure spheres are manually selected before continuing.');
end

magentaX = magentaCenters(:, 1);
magentaY = magentaCenters(:, 2);

%% Flat Surface Filtering
yValues = unique(magentaY);
xBorderMin = zeros(size(yValues));
xBorderMax = zeros(size(yValues));

for i = 1:length(yValues)
    currentY = yValues(i);
    indices = abs(magentaCenters(:, 2) - currentY) < tolerance;

    if any(indices)
        xBorderMin(i) = min(magentaCenters(indices, 1));
        xBorderMax(i) = max(magentaCenters(indices, 1));
    else
        if i > 1
            xBorderMin(i) = xBorderMin(i-1);
            xBorderMax(i) = xBorderMax(i-1);
        end
    end
end

xBorderMin = xBorderMin - tolerance;
xBorderMax = xBorderMax + tolerance;

interpXMin = interp1(yValues, xBorderMin, ptCloud.Location(:, 2), 'linear', 'extrap');
interpXMax = interp1(yValues, xBorderMax, ptCloud.Location(:, 2), 'linear', 'extrap');

validIndices = ...
    (ptCloud.Location(:, 1) >= interpXMin & ptCloud.Location(:, 1) <= interpXMax) & ...
    (ptCloud.Location(:, 2) >= min(yValues) - tolerance & ...
     ptCloud.Location(:, 2) <= max(yValues) + tolerance);

filteredPoints = ptCloud.Location(validIndices, :);

ptCloudFiltered = pointCloud(filteredPoints);

figure;
pcshow(ptCloudFiltered);
title('Filtered Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;

%% Smallest Notch Detection
numMeshSteps = 7; % Change
meshRefinementFactor = 2; % Change
minSphereRadius = 0.02; % Change
maxSphereRadius = 5.0; % Change

X = ptCloudFiltered.Location(:, 1);
Y = ptCloudFiltered.Location(:, 2);
Z = ptCloudFiltered.Location(:, 3);

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');

smallestSphereRadius = inf;
smallestSphereCenter = [];
smallestSphereStep = -1;

for step = 1:numMeshSteps
    gridResolution = 25 + step * meshRefinementFactor;
    [xMesh, yMesh] = meshgrid(linspace(min(X), max(X), gridResolution), ...
                              linspace(min(Y), max(Y), gridResolution));
    zMesh = F(xMesh, yMesh);

    for i = 1:size(xMesh, 1)
        for j = 1:size(xMesh, 2)
            center = [xMesh(i, j), yMesh(i, j), zMesh(i, j)];
            distances = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2);

            radiusCandidates = distances(distances > minSphereRadius & distances < maxSphereRadius);
            if length(radiusCandidates) < 3, continue; end

            validRadius = min(radiusCandidates);
            if validRadius < smallestSphereRadius
                smallestSphereRadius = validRadius;
                smallestSphereCenter = center;
                smallestSphereStep = step;
            end
        end
    end
end

disp(['Smallest sphere radius: ', num2str(smallestSphereRadius)]);
disp(['Smallest sphere center: [', num2str(smallestSphereCenter), ']']);
disp(['Found at mesh step: ', num2str(smallestSphereStep)]);

%% Displaying Smallest Notch (Does not work at the moment)
if ~isempty(smallestSphereCenter)
    % Add a large red arrow pointing to the smallest sphere center
    quiver3(smallestSphereCenter(1), smallestSphereCenter(2), smallestSphereCenter(3), ...
            0, 0, 10, 'Color', 'red', 'LineWidth', 3, 'MaxHeadSize', 5);
    % Add a red marker at the smallest sphere center
    plot3(smallestSphereCenter(1), smallestSphereCenter(2), smallestSphereCenter(3), ...
          'ro', 'MarkerSize', 15, 'LineWidth', 3);
else
    warning('No valid sphere found for visualization.');
end
hold off;
