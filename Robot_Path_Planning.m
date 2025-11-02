robot_original = [4 24;
                  12 24;
                  12 25;
                  4 25];

obstacle_1 = [0 18;
              10 18;
              10 19;
              0 19];

num_steps = 32;
angles_deg = linspace(0, 360 - 360/num_steps, num_steps);
center = mean(robot_original);
steps_to_show = [1, 8, 16, 32];

for s = 1:length(steps_to_show)
    step = steps_to_show(s);
    theta = angles_deg(step);
    angle_rad = deg2rad(theta);
    R = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    robot_centered = robot_original - center + [4, 0.5];
    robot_rotated = (R * robot_centered')';

    figure;
    hold on;

    % מכשול יחיד (obs = B1)
    obs = obstacle_1;
    CB_vertices = createCObstacle(robot_rotated, obs);

    if ~isempty(CB_vertices)
        axis equal;
        fill(obs(:,1), obs(:,2), 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r');  % ציור המכשול
        plot(CB_vertices(:,1), CB_vertices(:,2), '-', 'Color', 'r', 'LineWidth', 2);  % קווים בין הנקודות
        plot([CB_vertices(end,1), CB_vertices(1,1)], ...
             [CB_vertices(end,2), CB_vertices(1,2)], '-', 'Color', 'r', 'LineWidth', 2);  % חיבור קצה ראשון ואחרון
        title(sprintf('C-obstacle at θ = %.1f°', theta), 'FontWeight', 'bold');
        axis equal;
        grid on;
    end
end

%% *Q2*


obs_B1 = [0 18; 10 18; 10 19; 0 19];
obs_B2 = [17 17; 18 17; 18 29; 17 29];
obs_B3 = [27 18; 32 18; 32 19; 27 19];
obs_B4 = [0 14; 19 14; 19 15; 0 15];
obs_B5 = [24 13; 32 13; 32 15; 24 15];
obs_B6 =[10,19; 12,19; 12,20; 10,20];
obs_B7 = [25,19;27,19;27,20;25,20];
obs_B01 = [0 29; 32 29; 32 30; 0 30]; 
obs_B02 = [0 0; 1 0; 1 30; 0 30];   
obs_B03 = [0 0; 32 0; 32 1; 0 1];  
obs_B04 = [31 0; 32 0; 32 30; 31 30];

obstacles = {obs_B1, obs_B2, obs_B3, obs_B4, obs_B5, obs_B6,obs_B7,obs_B01, obs_B02, obs_B03, obs_B04};
obstacle_names = {'B1', 'B2', 'B3', 'B4', 'B5','B6' ,'B7','B01', 'B02', 'B03', 'B04'};
colors = lines(length(obstacles));

for s = 1:length(steps_to_show)
    step = steps_to_show(s);
    theta = angles_deg(step);
    angle_rad = deg2rad(theta);
    R = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    robot_centered = robot_original - center+[4,1/2];
    robot_rotated = (R * robot_centered')' ;

    %% תרשים 1 : ללא מרחב מכשול
    figure
    hold on;
    center = mean(robot_original);
    robot_centered = robot_original - center;
    robot_rotated_no_shift = (R * robot_centered')' + center;
    
    fill(robot_rotated_no_shift(:,1), robot_rotated_no_shift(:,2),[0.2 0.5 1], 'FaceAlpha', 0.4, 'EdgeColor', 'b', 'LineWidth', 2);
    for j = 1:length(obstacles)
        obs = obstacles{j};
        fill(obs(:,1), obs(:,2), colors(j,:), 'FaceAlpha', 0.15, 'EdgeColor', colors(j,:))
    end

    xlim([0 32]);
    ylim([0 32]);
    draw_grid_32x32([0 32], [0 32]);
    title(sprintf('Step %d – θ = %.1f° ', step, theta), 'FontWeight', 'bold');
    axis equal; grid on; hold off;

   %% תרשים 2 : עם מרחב מכשול
    figure
    hold on;
    for j = 1:length(obstacles)
        obs = obstacles{j};
        CB_vertices = createCObstacle(robot_rotated, obs);
    
        if ~isempty(CB_vertices)
            fill(obs(:,1), obs(:,2), colors(j,:), 'FaceAlpha', 0.1, 'EdgeColor', colors(j,:));
            plot(CB_vertices(:,1), CB_vertices(:,2), '-', 'Color', colors(j,:), 'LineWidth', 2);
            plot([CB_vertices(end,1), CB_vertices(1,1)], [CB_vertices(end,2), CB_vertices(1,2)], '-', 'Color', colors(j,:), 'LineWidth', 2);
        end
    end
    
    % ציור הרשת והכותרת
    draw_grid_32x32([0 32], [0 32]);
    title(sprintf('Step %d : θ = %.1f° (C-obstacles)', step, theta), 'FontWeight', 'bold');
    grid on; hold off;

end

%% *Q3*


    % פרמטרים כלליים
    grid_size = 32;
    num_steps = 32;
    room_x_min = 0; room_x_max = 32;
    room_y_min = 0; room_y_max = 32;
    angles_deg = linspace(0, 360 - 360/num_steps, num_steps);
    center = mean(robot_original);
    CBs_all_layers = cell(num_steps, length(obstacles));

    for step = 1:num_steps
        theta = angles_deg(step);
        angle_rad = deg2rad(theta);
        R = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
        robot_centered = robot_original - center + [4, 0.5];
        rotated_robot = (R * robot_centered')';

        for obs_idx = 1:length(obstacles)
            obstacle = obstacles{obs_idx};
            CB_vertices = createCObstacle(rotated_robot, obstacle);
            if ~isempty(CB_vertices)
                CBs_all_layers{step, obs_idx} = CB_vertices;
            end
        end
    end

    % בניית מטריצת C-space תלת-ממדית (x, y, θ)
    grid_3d = zeros(grid_size, grid_size, num_steps);
    for step = 1:num_steps
        for obs_idx = 1:length(obstacles)
            CB_vertices = CBs_all_layers{step, obs_idx};
            if ~isempty(CB_vertices)
                grid_3d(:,:,step) = matrix_fill(grid_3d(:,:,step), CB_vertices, room_x_min, room_x_max, room_y_min, room_y_max);
            end
        end
    end

    % שכבות להצגה
    layers_to_display = [1, 8, 16, 32];

    for idx = 1:length(layers_to_display)
        layer_num = layers_to_display(idx);
        theta = angles_deg(layer_num);

        figure('Position', [100 + layer_num*100, 100 + layer_num*50, 1000, 800]);
        hold on;

        % ציור הריבועים בשחור-לבן (הרשת הדיסקרטית)
        grid_matrix = grid_3d(:,:,layer_num);
        for j = 1:grid_size
            for k = 1:grid_size
                x_left = room_x_min + (k-1) * (room_x_max - room_x_min) / grid_size;
                x_right = room_x_min + k * (room_x_max - room_x_min) / grid_size;
                y_bottom = room_y_min + (j-1) * (room_y_max - room_y_min) / grid_size;
                y_top = room_y_min + j * (room_y_max - room_y_min) / grid_size;

                if grid_matrix(j, k) == 1
                    fill([x_left, x_right, x_right, x_left],[y_bottom, y_bottom, y_top, y_top], [0, 0, 0]);
                else
                    fill([x_left, x_right, x_right, x_left], [y_bottom, y_bottom, y_top, y_top], [1, 1, 1]);
                end
            end
        end

        for k = 0:grid_size
            x_line = room_x_min + k * (room_x_max - room_x_min) / grid_size;
            plot([x_line, x_line], [room_y_min, room_y_max], '-', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.5);
        end
        for j = 0:grid_size
            y_line = room_y_min + j * (room_y_max - room_y_min) / grid_size;
            plot([room_x_min, room_x_max], [y_line, y_line], '-', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.5);
        end

        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            fill(obs(:,1), obs(:,2), [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        end
        colors = lines(length(obstacles));

        % ציור ה-C-obstacles עם צבעים שונים
        for obs_idx = 1:length(obstacles)
            CB_vertices = CBs_all_layers{layer_num, obs_idx};
            if ~isempty(CB_vertices) && size(CB_vertices,1) >= 3
              try
                k = convhull(CB_vertices(:,1), CB_vertices(:,2));
                hull_pts = CB_vertices(k, :);
                plot(hull_pts(:,1), hull_pts(:,2), '-', 'Color', colors(obs_idx,:), 'LineWidth', 2);
            catch ME
                warning('Failed to draw obstacle %d: %s', obs_idx, ME.message);
            end

            end
        end

        xlim([0, 32]);
        ylim([0, 32]);
        axis equal;

        title(sprintf('Discretized C-Space Grid - Layer %d: θ = %.2f°', layer_num, theta), 'FontSize', 14, 'FontWeight', 'bold');
        xlabel('X [m]', 'FontSize', 12);
        ylabel('Y [m]', 'FontSize', 12);

        hold off;
    end

figure('Position', [500, 200, 1000, 800]);
hold on;

[X, Y] = meshgrid(1:grid_size, 1:grid_size);
[X, Y] = deal(X - 0.5, Y - 0.5);

colors = lines(length(layers_to_display));

for idx = 1:length(layers_to_display)
    step = layers_to_display(idx);
    theta_deg = angles_deg(step);
    Z = theta_deg * ones(size(X)); 

    layer = grid_3d(:,:,step);
    [rows, cols] = find(layer == 1); 
    color = colors(idx, :); 

    for k = 1:length(rows)
        i = rows(k);
        j = cols(k);

        x = j - 1;
        y = i - 1;
        z = theta_deg;

        fill3([x x+1 x+1 x], [y y y+1 y+1], [z z z z], color, ...
              'FaceAlpha', 0.9, 'EdgeColor', 'none');
    end
end

xlabel('X'); ylabel('Y'); zlabel('\theta [deg]');
xlim([0, 32]); ylim([0, 32]); zlim([0, 360]);
view(45, 25); 
title('3D Discretized C-Space (Selected Layers)');
grid on;
axis tight;
hold off;


%%  *Q1: func creat Cobs* 

function CB_vertices = createCObstacle(robot, obstacle)

    robot_normals = get_normals(robot, true);
    obs_normals = get_normals(obstacle, false);

    overlaps = check_all_arc_overlaps(robot_normals, obs_normals);

    robot_points = struct();
    for i = 1:size(robot,1)
        robot_points.(sprintf('A%d', i)) = robot(i,:);
    end
    obs_points = struct();
    for i = 1:size(obstacle,1)
        obs_points.(sprintf('B%d', i)) = obstacle(i,:);
    end
    diffs_b_minus_a = compute_diff_vectors_b_minus_a(overlaps, robot_points, obs_points);
    center_shifted = mean(diffs_b_minus_a, 1);
    angles_points = atan2(diffs_b_minus_a(:,2) - center_shifted(2), diffs_b_minus_a(:,1) - center_shifted(1));
    [~, order] = sort(angles_points);
    CB_vertices = diffs_b_minus_a(order, :);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function normals = get_normals(polygon, invert)
    n = size(polygon, 1);
    normals = zeros(n, 2);
    for i = 1:n
        p1 = polygon(i, :);
        p2 = polygon(mod(i, n) + 1, :);
        edge = p2 - p1;
        normal = [edge(2), -edge(1)];
        if invert
            normal = -normal;
        end
        normals(i, :) = normal / norm(normal);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overlap_pairs = check_all_arc_overlaps(robot_normals, obstacle_normals)
    robot_pairs = [4 1; 1 2; 2 3; 3 4];
    robot_labels = {'a1', 'a2', 'a3', 'a4'}; 
    obs_pairs = [1 2; 2 3; 3 4; 4 1];
    obstacle_labels = {'b2', 'b3', 'b4', 'b1'};
    robot_angles = mod(rad2deg(atan2(robot_normals(:,2), robot_normals(:,1))), 360);
    obstacle_angles = mod(rad2deg(atan2(obstacle_normals(:,2), obstacle_normals(:,1))), 360);
    robot_arcs = struct();
    obstacle_arcs = struct();

    for i = 1:length(robot_labels)
        start_idx = robot_pairs(i,1);
        end_idx = robot_pairs(i,2);
        start_angle = robot_angles(start_idx);
        end_angle = robot_angles(end_idx);
        if end_angle <= start_angle
            end_angle = end_angle + 360;
        end
        robot_arcs.(robot_labels{i}) = [start_angle, end_angle];
    end

    for i = 1:length(obstacle_labels)
        start_idx = obs_pairs(i,1);
        end_idx = obs_pairs(i,2);
        start_angle = obstacle_angles(start_idx);
        end_angle = obstacle_angles(end_idx);
        if end_angle <= start_angle
            end_angle = end_angle + 360;
        end
        obstacle_arcs.(obstacle_labels{i}) = [start_angle, end_angle];
    end

    overlap_pairs = {}; 
    count = 0;
    for i = 1:length(robot_labels)
        a_label = robot_labels{i};
        a_range = robot_arcs.(a_label);

        for j = 1:length(obstacle_labels)
            b_label = obstacle_labels{j};
            b_range = obstacle_arcs.(b_label);

            if circular_overlap_full(a_range, b_range)
                count = count + 1;
                overlap_pairs{count, 1} = b_label;
                overlap_pairs{count, 2} = a_label;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function res = circular_overlap_full(a_range, b_range)
    a_start = mod(a_range(1), 360);
    a_end   = a_range(2);
    if a_end <= a_start
        a_end = a_end + 360;
    end

    b_start = mod(b_range(1), 360);
    b_end   = b_range(2);
    if b_end <= b_start
        b_end = b_end + 360;
    end

    a_options = [a_start, a_end; a_start+360, a_end+360];
    b_options = [b_start, b_end; b_start+360, b_end+360];

    res = false;
    for ai = 1:2
        for bi = 1:2
            a_s = a_options(ai,1);
            a_e = a_options(ai,2);
            b_s = b_options(bi,1);
            b_e = b_options(bi,2);
            if (a_s < b_e) && (b_s < a_e)
                res = true;
                return;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function diff_vectors = compute_diff_vectors_b_minus_a(overlap_pairs, robot_points, obstacle_points)
    n = size(overlap_pairs,1);
    diff_vectors = zeros(n, 2); % כל שורה וקטור (x,y)

    for k = 1:n
        b_label = overlap_pairs{k,1}; % למשל 'b1'
        a_label = overlap_pairs{k,2}; % למשל 'a1'
        
        % המרת שמות ל- struct keys
        b_key = ['B' upper(b_label(2))]; % 'b1' -> 'B1'
        a_key = ['A' upper(a_label(2))]; % 'a1' -> 'A1'
        
        b_point = obstacle_points.(b_key);
        a_point = robot_points.(a_key);
        
        diff_vectors(k,:) = b_point - a_point; % חישוב b - a במקום a - b
    end
end
%% *Q2: func draw_grid 32X32*

% פונקציה שמציירת רשת של 32×32 ריבועים במרחב הציור לפי טווח הצירים
function draw_grid_32x32(x_lim, y_lim)
    x_start = x_lim(1);
    x_end = x_lim(2);
    y_start = y_lim(1);
    y_end = y_lim(2);

    x_ticks = linspace(x_start, x_end, 33);
    y_ticks = linspace(y_start, y_end, 33);

    % קווים אנכיים
    for k = 1:length(x_ticks)
        xline(x_ticks(k), 'Color', [0.7 0.7 0.7], 'LineStyle', '-');
    end
    % קווים אופקיים
    for k = 1:length(y_ticks)
        yline(y_ticks(k), 'Color', [0.7 0.7 0.7], 'LineStyle', '-');
    end
end
%% *Q3: func : discretized straight line between two given endpoints*

function grid_layer = matrix_fill(grid_layer, CB_vertices, x_min, x_max, y_min, y_max)
    if size(CB_vertices, 1) >= 3
        try
            k = convhull(CB_vertices(:,1), CB_vertices(:,2));
            hull_vertices = CB_vertices(k, :);
        catch
            hull_vertices = [CB_vertices; CB_vertices(1,:)];
        end
    else
        hull_vertices = [CB_vertices; CB_vertices(1,:)];
    end
    for i = 1:size(hull_vertices,1)-1
        start_point = hull_vertices(i, :);
        end_point = hull_vertices(i+1, :);
        grid_layer = district_line(grid_layer, start_point, end_point, ...
            x_min, x_max, y_min, y_max);
    end
end

function grid_layer = district_line(grid_layer, start_point, end_point, x_min, x_max, y_min, y_max)
    start_x = (start_point(1) - x_min) / (x_max - x_min) * 32;
    start_y = (start_point(2) - y_min) / (y_max - y_min) * 32;
    end_x = (end_point(1) - x_min) / (x_max - x_min) * 32;
    end_y = (end_point(2) - y_min) / (y_max - y_min) * 32;

    if abs(round(end_x + 0.5) - round(start_x + 0.5)) < ...
       abs(round(end_y + 0.5) - round(start_y + 0.5))
        steps = abs(round(end_y + 0.5) - round(start_y + 0.5)) + 1;
    else
        steps = abs(round(end_x + 0.5) - round(start_x + 0.5)) + 1;
    end

    xs = round(linspace(start_x + 0.5, end_x + 0.5, steps));
    ys = round(linspace(start_y + 0.5, end_y + 0.5, steps));

    valid_idx = (xs >= 1) & (xs <= 32) & (ys >= 1) & (ys <= 32);
    xs = xs(valid_idx);
    ys = ys(valid_idx);

    for i = 1:length(xs)
        grid_layer(ys(i), xs(i)) = 1;
    end
end







%% ===== פרמטרים כלליים =====
room_x_min = 0; room_x_max = 32;
room_y_min = 0; room_y_max = 32;

% רשת עדינה מומלצת; אם כבד אפשר 64
grid_size = 64;
num_steps = 64;

angles_deg = (0:num_steps-1) * (360/num_steps);

% מרכז הגוף (Anchor) — עובדים סביבו בלבד
center = mean(robot_original,1);   % [cx, cy]

% מרכזי תאים (לעבודה עם inpolygon + המרות אינדקס->עולם)
x_edges   = linspace(room_x_min, room_x_max, grid_size+1);
y_edges   = linspace(room_y_min, room_y_max, grid_size+1);
x_centers = 0.5*(x_edges(1:end-1)+x_edges(2:end));
y_centers = 0.5*(y_edges(1:end-1)+y_edges(2:end));
[XX,YY]   = meshgrid(x_centers, y_centers);

%% ===== חישוב C-obstacles (Minkowski בדיסקרטי) =====
CBs_all_layers = cell(num_steps, length(obstacles));
for step = 1:num_steps
    theta = angles_deg(step);
    R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];

    % חשוב: בלי היסטים לעולם — רק מערכת-גוף סביב המרכז
    robot_centered = robot_original - center;
    rotated_robot  = (R * robot_centered')';

    for obs_idx = 1:length(obstacles)
        obstacle = obstacles{obs_idx};     % מכשולים בקואורדינטות עולם
        CB_vertices = createCObstacle(rotated_robot, obstacle);  % b - a
        if ~isempty(CB_vertices)
            % ליציבות: מעטפת קמורה של קבוצת ההפרשים
            if size(CB_vertices,1) >= 3
                try
                    k = convhull(CB_vertices(:,1), CB_vertices(:,2));
                    CB_hull = CB_vertices(k(1:end-1), :);
                catch
                    CB_hull = unique(CB_vertices, 'rows');
                end
            else
                CB_hull = unique(CB_vertices, 'rows');
            end
            CBs_all_layers{step, obs_idx} = CB_hull;
        end
    end
end

%% ===== בניית מטריצת C-space (row=y, col=x, θ) =====
grid_3d = false(grid_size, grid_size, num_steps);   % 1=אסור, 0=חופשי
for step = 1:num_steps
    layer_mask = false(grid_size, grid_size);
    for obs_idx = 1:length(obstacles)
        CB_hull = CBs_all_layers{step, obs_idx};
        if ~isempty(CB_hull)
            inCB = inpolygon(XX, YY, CB_hull(:,1), CB_hull(:,2));
            layer_mask = layer_mask | inCB;
        end
    end
    grid_3d(:,:,step) = layer_mask;
end

%% ===== אתחול A* (מיפוי התחלה/מטרה מעולם לגריד) =====
S_world = [4, 24, 0];   % [x,y,theta_deg] בעולם (מרכז הרובוט)
T_world = [4,  8, 0];

[ny, nx, ntheta] = size(grid_3d);

[ixs, iys] = world2grid(S_world(1), S_world(2), x_edges, y_edges);
[ixg, iyg] = world2grid(T_world(1), T_world(2), x_edges, y_edges);
its = ang2bin(S_world(3), ntheta) - 1;   % 0..K-1 לשמירה על הקוד שלך
itg = ang2bin(T_world(3), ntheta) - 1;

S = [iys, ixs, its];   % [row, col, theta_index(0-based)]
T = [iyg, ixg, itg];

% וידוא שלא נופלים על תא חסום; אם כן – החלף לתא חופשי קרוב באותה שכבת θ
if grid_3d(S(1),S(2),S(3)+1)
    [S, ~] = snapXY_sameTheta_ifBlocked(grid_3d, S, true);
end
if grid_3d(T(1),T(2),T(3)+1)
    [T, ~] = snapXY_sameTheta_ifBlocked(grid_3d, T, true);
end

%% ===== מטריצות עלויות ו־Open/Closed =====
g = inf(ny, nx, ntheta);
h = zeros(ny, nx, ntheta);
f = inf(ny, nx, ntheta);
parent = cell(ny, nx, ntheta);

heuristic = @(node, Goal) hypot(node(1)-Goal(1), node(2)-Goal(2)); % XY בלבד (אפשר לשדרג)

g(S(1), S(2), S(3)+1) = 0;
h(S(1), S(2), S(3)+1) = heuristic(S, T);
f(S(1), S(2), S(3)+1) = h(S(1), S(2), S(3)+1);

O = S;   % open list
C = [];  % closed list
path = [];

%% ===== לולאת A* =====
while ~isempty(O)
    % בחירת צומת עם f מינימלי
    f_values = zeros(size(O,1),1);
    for idx = 1:size(O,1)
        node = O(idx,:);
        f_values(idx) = f(node(1), node(2), node(3)+1);
    end
    [~, best_idx] = min(f_values);
    current = O(best_idx,:);  % [r,c,k0]
    O(best_idx,:) = [];
    C = [C; current];

    % יעד?
    if isequal(current, T)
        path = reconstruct_path(parent, current);
        break;
    end

    % שכנים (6-שכנות: 4 ב-XY + θ±1) עם עטיפת זווית
    neigh = [ current(1)+1, current(2),   current(3);
              current(1)-1, current(2),   current(3);
              current(1),   current(2)+1, current(3);
              current(1),   current(2)-1, current(3);
              current(1),   current(2),   current(3)+1;
              current(1),   current(2),   current(3)-1 ];
    for n = 1:size(neigh,1)
        rr = neigh(n,1);
        cc = neigh(n,2);
        kk = mod(neigh(n,3), ntheta);   % wrap 0..K-1

        % גבולות XY
        if rr < 1 || rr > ny || cc < 1 || cc > nx
            continue;
        end

        % חסום?
        if grid_3d(rr, cc, kk+1)
            continue;
        end

        % כבר ב־Closed?
        if ismember([rr,cc,kk], C, 'rows')
            continue;
        end

        % עלות צעד (פשוטה; אפשר לשפר לפיזית)
        step_cost = 1;
        tentative_g = g(current(1), current(2), current(3)+1) + step_cost;

        if tentative_g < g(rr,cc,kk+1)
            g(rr,cc,kk+1) = tentative_g;
            h(rr,cc,kk+1) = heuristic([rr,cc,kk], T);
            f(rr,cc,kk+1) = g(rr,cc,kk+1) + h(rr,cc,kk+1);
            parent{rr,cc,kk+1} = current;

            if ~ismember([rr,cc,kk], O, 'rows')
                O = [O; [rr,cc,kk]];
            end
        end
    end
end

if isempty(path)
    disp('No path found!');
else
    disp('Target reached!');
end

%% ===== ויזואליזציה 2D של כל שלבי המסלול =====
figure('Color','w'); hold on; axis equal tight;
xlim([room_x_min room_x_max]); ylim([room_y_min room_y_max]);
set(gca,'YDir','normal'); grid on;
title('Robot path in 2D with full silhouette per step');

% מציירים מכשולים
for j = 1:length(obstacles)
    obs = obstacles{j};
    patch(obs(:,1), obs(:,2), [0.85 0.85 0.85], 'EdgeColor',[0.3 0.3 0.3], 'LineWidth',1.2);
end

% קו המסלול (היטל XY)
if ~isempty(path)
    px = x_centers(path(:,2));
    py = y_centers(path(:,1));
    plot(px, py, 'r-', 'LineWidth', 2);
end

% סילואטת הרובוט בכל צעד
robot_local = robot_original - center;  % מערכת-גוף
for s = 1:size(path,1)
    xw = x_centers(path(s,2));
    yw = y_centers(path(s,1));
    thetai0 = path(s,3);                       % 0..K-1
    theta_deg = angles_deg(thetai0+1);
    R = [cosd(theta_deg) -sind(theta_deg); sind(theta_deg) cosd(theta_deg)];
    robot_world = (R * robot_local')' + [xw, yw];

    patch(robot_world(:,1), robot_world(:,2), [0.2 0.5 1], ...
          'FaceAlpha', 0.20, 'EdgeColor', [0.2 0.5 1]);
    % לצפייה כאנימציה: בטלי את ההערה הבאה
    % drawnow; pause(0.01);
end

% התחלה/מטרה (בעולם)
plot(S_world(1), S_world(2), 'go', 'MarkerFaceColor','g', 'MarkerSize', 9, 'LineWidth',1.2);
plot(T_world(1), T_world(2), 'ro', 'MarkerFaceColor','r', 'MarkerSize', 9, 'LineWidth',1.2);
legend({'Obstacles','Path (XY)','Robot silhouette','Start','Goal'}, 'Location','best');

%% ===== פונקציות עזר =====
function [ix, iy] = world2grid(x, y, x_edges, y_edges)
    xc = 0.5*(x_edges(1:end-1)+x_edges(2:end));
    yc = 0.5*(y_edges(1:end-1)+y_edges(2:end));
    [~, ix] = min(abs(x - xc));
    [~, iy] = min(abs(y - yc));
end

function ib = ang2bin(theta_deg, num_bins)
    theta_deg = mod(theta_deg, 360);
    ib = floor(theta_deg/360 * num_bins) + 1;   % 1..num_bins
end

function path = reconstruct_path(parent, current)
    path = current;
    while true
        p = parent{current(1), current(2), current(3)+1};
        if isempty(p), break; end
        path = [p; path];
        current = p;
    end
end

function [rck_out, didSnap] = snapXY_sameTheta_ifBlocked(gridC, rck_in, allow)
    didSnap = false; rck_out = rck_in;
    if ~allow, return; end
    [H,W,~] = size(gridC);
    r = double(rck_in(1)); c = double(rck_in(2)); k = double(rck_in(3))+1;
    if ~gridC(r,c,k), return; end
    maxR = max(H,W); best=[]; bestScore=inf;
    for rad = 1:maxR
        rmin = max(1,r-rad); rmax = min(H,r+rad);
        cmin = max(1,c-rad); cmax = min(W,c+rad);
        ring = [ (rmin:rmax)' repmat(cmin, rmax-rmin+1,1); ...
                 (rmin:rmax)' repmat(cmax, rmax-rmin+1,1); ...
                 repmat(rmin, cmax-cmin+1,1) (cmin:cmax)'; ...
                 repmat(rmax, cmax-cmin+1,1) (cmin:cmax)' ];
        for i = 1:size(ring,1)
            rr = ring(i,1); cc = ring(i,2);
            if ~gridC(rr,cc,k)
                score = max(abs(rr-r), abs(cc-c));
                if score < bestScore
                    bestScore = score; best = [rr,cc,k-1];
                end
            end
        end
        if ~isempty(best), break; end
    end
    if ~isempty(best), rck_out = best; didSnap = true; end
end


%%
function CB_vertices = createCObstacle(robot, obstacle)

    robot_normals = get_normals(robot, true);
    obs_normals = get_normals(obstacle, false);

    overlaps = check_all_arc_overlaps(robot_normals, obs_normals);

    robot_points = struct();
    for i = 1:size(robot,1)
        robot_points.(sprintf('A%d', i)) = robot(i,:);
    end
    obs_points = struct();
    for i = 1:size(obstacle,1)
        obs_points.(sprintf('B%d', i)) = obstacle(i,:);
    end
    diffs_b_minus_a = compute_diff_vectors_b_minus_a(overlaps, robot_points, obs_points);
    center_shifted = mean(diffs_b_minus_a, 1);
    angles_points = atan2(diffs_b_minus_a(:,2) - center_shifted(2), diffs_b_minus_a(:,1) - center_shifted(1));
    [~, order] = sort(angles_points);
    CB_vertices = diffs_b_minus_a(order, :);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function normals = get_normals(polygon, invert)
    n = size(polygon, 1);
    normals = zeros(n, 2);
    for i = 1:n
        p1 = polygon(i, :);
        p2 = polygon(mod(i, n) + 1, :);
        edge = p2 - p1;
        normal = [edge(2), -edge(1)];
        if invert
            normal = -normal;
        end
        normals(i, :) = normal / norm(normal);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overlap_pairs = check_all_arc_overlaps(robot_normals, obstacle_normals)
    robot_pairs = [4 1; 1 2; 2 3; 3 4];
    robot_labels = {'a1', 'a2', 'a3', 'a4'}; 
    obs_pairs = [1 2; 2 3; 3 4; 4 1];
    obstacle_labels = {'b2', 'b3', 'b4', 'b1'};
    robot_angles = mod(rad2deg(atan2(robot_normals(:,2), robot_normals(:,1))), 360);
    obstacle_angles = mod(rad2deg(atan2(obstacle_normals(:,2), obstacle_normals(:,1))), 360);
    robot_arcs = struct();
    obstacle_arcs = struct();

    for i = 1:length(robot_labels)
        start_idx = robot_pairs(i,1);
        end_idx = robot_pairs(i,2);
        start_angle = robot_angles(start_idx);
        end_angle = robot_angles(end_idx);
        if end_angle <= start_angle
            end_angle = end_angle + 360;
        end
        robot_arcs.(robot_labels{i}) = [start_angle, end_angle];
    end

    for i = 1:length(obstacle_labels)
        start_idx = obs_pairs(i,1);
        end_idx = obs_pairs(i,2);
        start_angle = obstacle_angles(start_idx);
        end_angle = obstacle_angles(end_idx);
        if end_angle <= start_angle
            end_angle = end_angle + 360;
        end
        obstacle_arcs.(obstacle_labels{i}) = [start_angle, end_angle];
    end

    overlap_pairs = {}; 
    count = 0;
    for i = 1:length(robot_labels)
        a_label = robot_labels{i};
        a_range = robot_arcs.(a_label);

        for j = 1:length(obstacle_labels)
            b_label = obstacle_labels{j};
            b_range = obstacle_arcs.(b_label);

            if circular_overlap_full(a_range, b_range)
                count = count + 1;
                overlap_pairs{count, 1} = b_label;
                overlap_pairs{count, 2} = a_label;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function res = circular_overlap_full(a_range, b_range)
    a_start = mod(a_range(1), 360);
    a_end   = a_range(2);
    if a_end <= a_start
        a_end = a_end + 360;
    end

    b_start = mod(b_range(1), 360);
    b_end   = b_range(2);
    if b_end <= b_start
        b_end = b_end + 360;
    end

    a_options = [a_start, a_end; a_start+360, a_end+360];
    b_options = [b_start, b_end; b_start+360, b_end+360];

    res = false;
    for ai = 1:2
        for bi = 1:2
            a_s = a_options(ai,1);
            a_e = a_options(ai,2);
            b_s = b_options(bi,1);
            b_e = b_options(bi,2);
            if (a_s < b_e) && (b_s < a_e)
                res = true;
                return;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function diff_vectors = compute_diff_vectors_b_minus_a(overlap_pairs, robot_points, obstacle_points)
    n = size(overlap_pairs,1);
    diff_vectors = zeros(n, 2); % כל שורה וקטור (x,y)

    for k = 1:n
        b_label = overlap_pairs{k,1}; % למשל 'b1'
        a_label = overlap_pairs{k,2}; % למשל 'a1'
        
        % המרת שמות ל- struct keys
        b_key = ['B' upper(b_label(2))]; % 'b1' -> 'B1'
        a_key = ['A' upper(a_label(2))]; % 'a1' -> 'A1'
        
        b_point = obstacle_points.(b_key);
        a_point = robot_points.(a_key);
        
        diff_vectors(k,:) = b_point - a_point; % חישוב b - a במקום a - b
    end
end
% פונקציה שמציירת רשת של 32×32 ריבועים במרחב הציור לפי טווח הצירים
function draw_grid_32x32(x_lim, y_lim)
    x_start = x_lim(1);
    x_end = x_lim(2);
    y_start = y_lim(1);
    y_end = y_lim(2);

    x_ticks = linspace(x_start, x_end, 33);
    y_ticks = linspace(y_start, y_end, 33);

    % קווים אנכיים
    for k = 1:length(x_ticks)
        xline(x_ticks(k), 'Color', [0.7 0.7 0.7], 'LineStyle', '-');
    end
    % קווים אופקיים
    for k = 1:length(y_ticks)
        yline(y_ticks(k), 'Color', [0.7 0.7 0.7], 'LineStyle', '-');
    end
end
function grid_layer = matrix_fill(grid_layer, CB_vertices, x_min, x_max, y_min, y_max)
    if size(CB_vertices, 1) >= 3
        try
            k = convhull(CB_vertices(:,1), CB_vertices(:,2));
            hull_vertices = CB_vertices(k, :);
        catch
            hull_vertices = [CB_vertices; CB_vertices(1,:)];
        end
    else
        hull_vertices = [CB_vertices; CB_vertices(1,:)];
    end
    for i = 1:size(hull_vertices,1)-1
        start_point = hull_vertices(i, :);
        end_point = hull_vertices(i+1, :);
        grid_layer = district_line(grid_layer, start_point, end_point, ...
            x_min, x_max, y_min, y_max);
    end
end

function grid_layer = district_line(grid_layer, start_point, end_point, x_min, x_max, y_min, y_max)
    % ממפה קו לגריד בגודל 32x32
    start_x = (start_point(1) - x_min) / (x_max - x_min) * 64;
    start_y = (start_point(2) - y_min) / (y_max - y_min) * 32;
    end_x   = (end_point(1) - x_min) / (x_max - x_min) * 32;
    end_y   = (end_point(2) - y_min) / (y_max - y_min) * 32;

    % קובע את מספר הצעדים לפי הציר הדומיננטי
    if abs(round(end_x + 0.5) - round(start_x + 0.5)) < ...
       abs(round(end_y + 0.5) - round(start_y + 0.5))
        steps = abs(round(end_y + 0.5) - round(start_y + 0.5)) + 1;
    else
        steps = abs(round(end_x + 0.5) - round(start_x + 0.5)) + 1;
    end

    xs = round(linspace(start_x + 0.5, end_x + 0.5, steps));
    ys = round(linspace(start_y + 0.5, end_y + 0.5, steps));

    valid_idx = (xs >= 1) & (xs <= 32) & (ys >= 1) & (ys <= 32);
    xs = xs(valid_idx);
    ys = ys(valid_idx);

    for i = 1:length(xs)
        grid_layer(ys(i), xs(i)) = 1;
    end
end