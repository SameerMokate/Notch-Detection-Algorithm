%% Clear previous variables and figures
clearvars;
close all;
clc;

%% Model

model = stlread('D:\Work\VAKA - Work\WeldScanAlgorithm\butt_weld_cropped.stl');

X = model.Points(:, 1);
Y = model.Points(:, 2);
Z = model.Points(:, 3);
ptCloud = pointCloud([X, Y, Z]);

F = scatteredInterpolant(X, Y, Z, 'natural', 'none');

figure;
trisurf(model.ConnectivityList, X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none');
title('3D Model');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
camlight;
lighting gouraud;

%% Grid

gridResolution = 25;

xMin = min(ptCloud.Location(:,1));
xMax = max(ptCloud.Location(:,1));
yMin = min(ptCloud.Location(:,2));
yMax = max(ptCloud.Location(:,2));

[x, y] = meshgrid(linspace(xMin, xMax, gridResolution), ...
                  linspace(yMin, yMax, gridResolution));

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

%% Sphere

sphereRadius = 1;

verticalTolerance = 0.1;

figureHandle = figure;
pcshow(ptCloud);
hold on;

sphereHandles = gobjects(size(x));

for i = 1:size(x, 1)
    for j = 1:size(x, 2)

        center = [x(i, j), y(i, j), z(i, j)];

        distances = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2);
        
        verticalSeamInteraction = any(distances < sphereRadius & abs(Z - center(3)) > verticalTolerance);

        if verticalSeamInteraction
            sphereColor = 'magenta';
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

%% Removing Flat Surface
