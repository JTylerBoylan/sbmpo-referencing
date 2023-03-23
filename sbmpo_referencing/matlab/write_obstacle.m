%%
% Jonathan Boylan

function write_obstacle(x, y, r, runs)

obstacles = repmat([1 x y r], [runs 1]);

writematrix(obstacles, '../csv/obstacles.csv', 'Delimiter', ',');

end