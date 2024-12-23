%% Clear previous variables and figures
clearvars;
close all;
clc;

%% Load STL Model
model = stlread('D:\Work\VAKA - Work\WeldScanAlgorithm\butt_weld_cropped.stl');
X = model.Points(:, 1);
Y = model.Points(:, 2);
Z = model.Points(:, 3);
ptCloud = pointCloud([X, Y, Z]);

% Display the original point cloud
figure;
trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
title('3D Model');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
camlight;
lighting gouraud;

%% Generate Grid for Initial Magenta Sphere Selection
gridResolution = 25;
xMin = min(ptCloud.Location(:,1));
xMax = max(ptCloud.Location(:,1));
yMin = min(ptCloud.Location(:,2));
yMax = max(ptCloud.Location(:,2));

[x, y] = meshgrid(linspace(xMin, xMax, gridResolution), ...
                  linspace(yMin, yMax, gridResolution));

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');
z = F(x, y);

% Plot the grid over the point cloud
figure;
pcshow(ptCloud);
hold on;
for i = 1:gridResolution
    plot3(x(:, i), y(:, i), z(:, i), 'k-', 'LineWidth', 1.5);
    plot3(x(i, :), y(i, :), z(i, :), 'k-', 'LineWidth', 1.5);
end
plot3(x(:), y(:), z(:), 'r.', 'MarkerSize', 10);
title('3D Grid Over Weld Seam Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Sphere Placement and Interaction
sphereRadius = 1;
verticalTolerance = 0.1;

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
            magentaCenters = [magentaCenters; center]; %#ok<AGROW>
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

% Manual Selection Button
uicontrol('Style', 'pushbutton', 'String', 'Continue', ...
          'Position', [20, 20, 100, 40], 'Callback', @(~,~) uiresume(figureHandle));

uiwait(figureHandle);

% Toggle sphere color callback
function toggleSphereColor(sphere)
    currentColor = sphere.FaceColor;
    if isequal(currentColor, [1, 0, 1]) % Magenta
        sphere.FaceColor = 'black';
    else
        sphere.FaceColor = [1, 0, 1];
    end
end

%% Use Convex Hull to Define Weld Seam Region
% Compute convex hull of magenta sphere centers
k = convhull(magentaCenters(:, 1), magentaCenters(:, 2), magentaCenters(:, 3));

% Visualize convex hull
figure;
trisurf(k, magentaCenters(:, 1), magentaCenters(:, 2), magentaCenters(:, 3), ...
    'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'cyan');
hold on;
scatter3(magentaCenters(:, 1), magentaCenters(:, 2), magentaCenters(:, 3), ...
    50, 'magenta', 'filled');
title('Convex Hull of Magenta Sphere Centers');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off;

% Check Points Inside Convex Hull
inHull = inhull(ptCloud.Location, magentaCenters, k);

% Filter points inside the convex hull
filteredPoints = ptCloud.Location(inHull, :);
ptCloudFiltered = pointCloud(filteredPoints);

% Visualize filtered point cloud
figure;
pcshow(ptCloudFiltered);
title('Filtered Point Cloud: Weld Seam Region');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Convex Hull Helper Function
function in = inhull(testpts, xyz, tess, tol)
    % testpts: Nx3 array of points to test
    % xyz: Points forming the vertices of the convex hull
    % tess: Tessellation (output from `convhull`)
    % tol: Tolerance (optional)
    
    if nargin < 4
        tol = 1e-12; % Set a very small default tolerance
    end
    
    % Normal vectors for each face of the convex hull
    normals = cross(xyz(tess(:, 2), :) - xyz(tess(:, 1), :), ...
                    xyz(tess(:, 3), :) - xyz(tess(:, 1), :), 2);
    normals = normals ./ vecnorm(normals, 2, 2); % Normalize vectors
    
    % Points to check, shifted relative to hull vertices
    vecs = bsxfun(@minus, permute(testpts, [1 3 2]), permute(xyz(tess(:, 1), :), [3 1 2]));
    % Dot product of normals with vectors
    dotprods = sum(bsxfun(@times, vecs, permute(normals, [3 1 2])), 3);
    
    % Test points are inside if all dot products are <= tolerance
    in = all(dotprods <= tol, 2);
end
