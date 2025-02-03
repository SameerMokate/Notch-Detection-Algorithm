clearvars;
close all;
clc;
%% Load the Model
model = stlread('D:\Work\VAKA - Work\Notch Detection Algorithm\butt_weld_cropped1.stl');
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
        [sx, sy, sz] = sphere; %#ok<*RHSFN>
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
            magentaCenters = [magentaCenters; center]; %#ok<*AGROW>
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
hold on;
%% **Find Smallest 100 Notches on the Weld Seam**
numMeshSteps = 10;
meshRefinementFactor = 2;
minSphereRadius = 0.25;
maxSphereRadius = 5.0;
numSmallestSpheres = 100; % Keep track of the 100 smallest spheres

X = ptCloudFiltered.Location(:, 1);
Y = ptCloudFiltered.Location(:, 2);
Z = ptCloudFiltered.Location(:, 3);

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');

% Store the smallest 100 spheres
smallestSpheres = [];

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
            if length(radiusCandidates) < 20, continue; end

            validRadius = min(radiusCandidates);
            
            % Store this sphere if it's within the smallest 100
            smallestSpheres = [smallestSpheres; struct('radius', validRadius, 'center', center)];
            
            % Keep only the smallest 100 by sorting and trimming
            if length(smallestSpheres) > numSmallestSpheres
                [~, sortIdx] = sort([smallestSpheres.radius]);
                smallestSpheres = smallestSpheres(sortIdx(1:numSmallestSpheres));
            end
        end
    end
end

%% **Command Line Output**
%disp('Smallest 100 Spheres Found:');
%for k = 1:length(smallestSpheres)
%    disp(['Radius: ', num2str(smallestSpheres(k).radius), ...
%          ', Center: [', num2str(smallestSpheres(k).center), ']']);
%end

%% **Final Display - 100 Smallest Spheres on the Weld Seam**
% Plot all 100 smallest spheres using scatter3
for k = 1:length(smallestSpheres)
    scatter3(smallestSpheres(k).center(1), ...
             smallestSpheres(k).center(2), ...
             smallestSpheres(k).center(3), ...
             25, 'w', 'filled'); % Red markers for all small spheres
end

title('Top 100 Smallest Spheres (Notches) on Weld Seam');
xlabel('X'); ylabel('Y'); zlabel('Z');

hold off;
