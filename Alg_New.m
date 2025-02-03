%% Clear previous variables and figures
clearvars;
close all;
clc;

%% Load the Model
model = stlread('D:\Work\VAKA - Work\WeldScanAlgorithm\butt_weld_cropped1.stl');
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

figureHandle = figure;
trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
title('Weld Seam (Magenta) vs Flat Surface (Black)');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
camlight;
lighting gouraud;
hold on;

%% Curvature-Based Weld Seam Detection
curvatureThreshold = 0.01; % Curvature threshold to classify weld seam vs flat surface
meshColors = zeros(size(gridPoints, 1), 3); % RGB colors for the mesh points
curvatureValues = []; % Store curvature for each point

for i = 1:size(x, 1)
    % Extract mesh line points
    linePoints = [x(i, :)', y(i, :)', z(i, :)'];

    % Compute curvature for the line
    curvatures = calculateCurvature(linePoints);

    % Classify points
    for j = 1:size(linePoints, 1)
        if j > 1 && j < size(linePoints, 1) - 1
            if curvatures(j - 1) > curvatureThreshold
                % Weld seam
                meshColors((i - 1) * size(x, 2) + j, :) = [1, 0, 1]; % Magenta
            else
                % Flat surface
                meshColors((i - 1) * size(x, 2) + j, :) = [0, 0, 0]; % Black
            end
            curvatureValues = [curvatureValues; curvatures(j - 1)]; %#ok<AGROW>
        else
            % Boundary points default to flat surface
            meshColors((i - 1) * size(x, 2) + j, :) = [0, 0, 0]; % Black
        end
    end
end

% Plot mesh points over the geometry
sphereHandles = gobjects(size(gridPoints, 1), 1);
for i = 1:size(gridPoints, 1)
    sphereHandles(i) = scatter3(gridPoints(i, 1), gridPoints(i, 2), gridPoints(i, 3), ...
                                50, meshColors(i, :), 'filled', ...
                                'ButtonDownFcn', @(src, ~) toggleSphereColor(src, i));
end

uicontrol('Style', 'pushbutton', 'String', 'Continue', ...
          'Position', [20, 20, 100, 40], 'Callback', @(~,~) uiresume(figureHandle));

uiwait(figureHandle);

% Finalize adjusted weld seam points
adjustedWeldSeam = all(meshColors == [1, 0, 1], 2);
adjustedWeldSeamPoints = gridPoints(adjustedWeldSeam, :);

%% Sphere Placement (Smallest Sphere Detection)
numMeshSteps = 7; % Change
meshRefinementFactor = 2; % Change
minSphereRadius = 0.02; % Change
maxSphereRadius = 5.0; % Change

X = adjustedWeldSeamPoints(:, 1);
Y = adjustedWeldSeamPoints(:, 2);
Z = adjustedWeldSeamPoints(:, 3);

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

%% Visualizing Smallest Sphere
if ~isempty(smallestSphereCenter)
    figure;
    trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
    hold on;
    [sx, sy, sz] = sphere;
    sx = sx * smallestSphereRadius + smallestSphereCenter(1);
    sy = sy * smallestSphereRadius + smallestSphereCenter(2);
    sz = sz * smallestSphereRadius + smallestSphereCenter(3);
    surf(sx, sy, sz, 'FaceColor', 'magenta', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    title('Smallest Sphere (Notch) on Weld Seam');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    hold off;
end

%% Toggle Function for Manual Selection
function toggleSphereColor(src, index)
    currentColor = src.CData;
    if isequal(currentColor, [1, 0, 1]) % Magenta
        meshColors(index, :) = [0, 0, 0]; % Change to black
        src.CData = [0, 0, 0];
    else % Black
        meshColors(index, :) = [1, 0, 1]; % Change to magenta
        src.CData = [1, 0, 1];
    end
end

%% Curvature Calculation Function
function curvatureValues = calculateCurvature(points)
    dx = diff(points(:, 1));
    dy = diff(points(:, 2));
    dz = diff(points(:, 3));

    ddx = diff(dx);
    ddy = diff(dy);
    ddz = diff(dz);

    dx_mid = dx(1:end-1);
    dy_mid = dy(1:end-1);
    dz_mid = dz(1:end-1);

    num = sqrt((ddy .* dz_mid - ddz .* dy_mid).^2 + ...
               (ddz .* dx_mid - ddx .* dz_mid).^2 + ...
               (ddx .* dy_mid - ddy .* dx_mid).^2);
    denom = (dx_mid.^2 + dy_mid.^2 + dz_mid.^2).^(3/2);

    denom(denom == 0) = inf;
    curvatureValues = num ./ denom;
end
