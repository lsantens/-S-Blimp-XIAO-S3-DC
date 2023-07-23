% Clear the workspace and close all figures
clear;
close all;

% Replace 'data.csv' with the name of your CSV file if it's different
filename = 'globalTrack1690066700.7698867_-100.csv';

% Import the data from the CSV file
data = readtable(filename);

% Extract time, x, y, and z data from the table
time = data.time;
x = data.x;
y = data.y;
z = data.z;

% Create a 3D line plot
figure;
plot3(x, y, z, 'b-', 'LineWidth', 2);
hold off
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Line Plot');
xlim([-5, 7]);
ylim([-4, 4]);
zlim([0, 3]);
hold on
plot3(-1, 0, 1.6, 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r');