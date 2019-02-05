function [aligned_scans aligned_camera_views alignments obj_cloud] = ...
    align_scans(scans, camera_views, resolution, interactive, use_occ, use_box)
%[aligned_scans aligned_camera_views alignments obj_cloud] = ...
%    align_scans(scans, camera_views, resolution, interactive, use_occ, use_box)

if nargin < 4
    interactive = 0;
end
if nargin < 5
    use_occ = 0;
end
if nargin < 6
    use_box = 1;
end

c = colormap('jet');



% find the first non-empty scan
alignments = {};
aligned_scans = {};
for i0=1:length(scans)
    alignments{i0} = [0;0;0;0];
    aligned_scans{i0} = scans{i0};
    if ~isempty(scans{i0})
        break;
    end
end

% create obj_cloud and box_cloud
h = find_supporting_plane_height(scans{i0});
box_cloud = scans{i0}(scans{i0}(:,3)<=h,:);
obj_cloud = scans{i0}(scans{i0}(:,3)>h,:);
prev_box = [];

if use_occ
    % create occupancy grid and update it with the first scan
    pmin = min(scans{i0});
    pmax = max(scans{i0});
    for i=i0+1:length(scans)
        if ~isempty(scans{i})
            pmin = min(pmin, min(scans{i}));
            pmax = max(pmax, max(scans{i}));
        end
    end
    occ_grid = new_occupancy_grid(pmin-resolution, pmax+resolution, resolution);
    occ_grid = update_occupancy_grid(occ_grid, scans{i0}, camera_views(i0,:));
end

% create aligned camera views
aligned_camera_views = zeros(length(scans), 3);
aligned_camera_views(i0,:) = camera_views(i0,:);


figure(2);
clf;
plot3(scans{i0}(:,1), scans{i0}(:,2), scans{i0}(:,3), '.', 'Color', c(1,:));
for i=i0+1:length(scans)
    
    if isempty(scans{i})
        alignments{i} = alignments{i-1};
        continue;
    end

    align_this_scan = 1;
    if interactive
        while 1
            action = input(sprintf('scan %d: align this scan? [y/n]:', i),'s');
            if action=='y'
                align_this_scan = 1;
                break
            elseif action=='n'
                align_this_scan = 0;
                break
            else
                fprintf('invalid action\n');
            end
        end
    end

    % fit a box to box_cloud
    if use_box
        if isempty(prev_box)
            box = fit_box(box_cloud(1:10:end,:));
        else
            box = fit_box(box_cloud(1:10:end,:), prev_box);
        end
        prev_box = box;
    end
    
    % split scan into object and box scans
    h = find_supporting_plane_height(scans{i});
    box_scan = scans{i}(scans{i}(:,3)<=h,:);
    obj_scan = scans{i}(scans{i}(:,3)>h,:);
        
    if align_this_scan

        %cost_fn = @(xxx) alignment_cost(xxx, aligned_scans{i-1}, scans{i});
        cost_fn1 = @(xxx) alignment_cost(xxx, obj_cloud, obj_scan);
        if use_occ
            cost_fn2 = @(xxx) .0001 * alignment_cost_occ(xxx, obj_cloud, obj_scan, occ_grid);
        end
        if use_box
            cost_fn3 = @(xxx) 5*alignment_cost_box(xxx, box, box_scan(1:5:end,:));
        end
        if use_occ && use_box
            cost_fn = @(xxx) cost_fn1(xxx) + cost_fn2(xxx) + cost_fn3(xxx);
        elseif use_occ
            cost_fn = @(xxx) cost_fn1(xxx) + cost_fn2(xxx);
        elseif use_box
            cost_fn = @(xxx) cost_fn1(xxx) + cost_fn3(xxx);
        else
            cost_fn = @(xxx) cost_fn1(xxx);
        end

        theta = alignments{i-1}(4);
        q = [cos(theta/2); 0; 0; sin(theta/2)];
        X = transform_point_cloud(alignments{i-1}(1:3), q, obj_scan);
        init_alignment = alignments{i-1} + [mean(obj_cloud) - mean(X), 0]';
        alignments{i} = init_alignment;

        while 1
            %options = optimset('TypicalX', [.1; .1; .1; 1]);
            %alignments{i} = fminsearch(cost_fn, init_alignment, options);
            alignments{i} = randomized_gradient_descent(alignments{i}, cost_fn);
            fprintf('\n');

            if use_occ && use_box
                cost = [cost_fn1(alignments{i}), cost_fn2(alignments{i}), cost_fn3(alignments{i})]
            elseif use_box
                cost = [cost_fn1(alignments{i}), cost_fn3(alignments{i})]
            elseif use_occ
                cost = [cost_fn1(alignments{i}), cost_fn2(alignments{i})]
            else
                cost = [cost_fn1(alignments{i})]
            end
        
            if ~interactive
                break
            else
                done_aligning = 0;
                while 1
                    action = input('action? [R/r/X/x/Y/y/Z/z/a/d]:','s');
                    if action=='R'
                        alignments{i}(4) = alignments{i}(4) + .1;
                    elseif action=='r'
                        alignments{i}(4) = alignments{i}(4) - .1;
                    elseif action=='X'
                        alignments{i}(1) = alignments{i}(1) + .01;
                    elseif action=='x'
                        alignments{i}(1) = alignments{i}(1) - .01;
                    elseif action=='Y'
                        alignments{i}(2) = alignments{i}(2) + .01;
                    elseif action=='y'
                        alignments{i}(2) = alignments{i}(2) - .01;
                    elseif action=='Z'
                        alignments{i}(3) = alignments{i}(3) + .01;
                    elseif action=='z'
                        alignments{i}(3) = alignments{i}(3) - .01;
                    elseif action=='a'
                        break;
                    elseif action=='d'
                        done_aligning = 1;
                        break;
                    else
                        fprintf('invalid action\n');
                        continue;
                    end
                    figure(3);
                    clf;
                    hold on;
                    theta = alignments{i}(4);
                    q = [cos(theta/2); 0; 0; sin(theta/2)];
                    cloud = transform_point_cloud(alignments{i}(1:3), q, scans{i});
                    plot3(obj_cloud(:,1), obj_cloud(:,2), obj_cloud(:,3), 'b.');
                    plot3(cloud(:,1), cloud(:,2), cloud(:,3), 'r.');
                    if use_box
                        plot_box(box);
                    end
                    hold off;
                end
                if done_aligning
                    break;
                end
            end
        end

        theta = alignments{i}(4);
        q = [cos(theta/2); 0; 0; sin(theta/2)];
        [aligned_scans{i} A] = transform_point_cloud(alignments{i}(1:3), q, scans{i});
        aligned_obj_scan = transform_point_cloud(alignments{i}(1:3), q, obj_scan);
        v = A*[camera_views(i,:) 1]';
        aligned_camera_views(i,:) = v(1:3)';

    else
        alignments{i} = [0;0;0;0];
        aligned_scans{i} = scans{i};
        aligned_obj_scan = obj_scan;
    end
    
    combined_cloud = [];
    for j=1:i
        combined_cloud = [combined_cloud ; aligned_scans{j}];
    end
    h = find_supporting_plane_height(combined_cloud);
    box_cloud = combined_cloud(combined_cloud(:,3)<=h,:);
    obj_cloud = combined_cloud(combined_cloud(:,3)>h,:);
    obj_cloud = downsample_point_cloud(obj_cloud, resolution);
    box_cloud = downsample_point_cloud(box_cloud, resolution);
    if use_occ
        occ_grid = update_occupancy_grid(occ_grid, aligned_scans{i}, aligned_camera_views(i,:));
    end
    
    % plot aligned scans
    clf
    for j=i0:i
        if ~isempty(aligned_scans{j})
            hold on;
            plot3(aligned_scans{j}(:,1), aligned_scans{j}(:,2), aligned_scans{j}(:,3), ...
                  '.', 'Color', c(mod(20*(j-1),size(c,1))+1,:));
            hold off;
        end
    end
    axis vis3d;
    axis equal;
    axis tight;
    drawnow;
    %input(':');
end



% get camera views
% for i=1:length(scans)
%     if ~isempty(scans{i})
%         theta = alignments{i}(4);
%         q = [cos(theta/2); 0; 0; sin(theta/2)];
%         [xxx A] = transform_point_cloud(alignments{i}(1:3), q, scans{i});
%         v = A*[camera_views(i,:) 1]';
%         aligned_camera_views(i,:) = v(1:3)';
%     end
% end


%cost_fn = @(w) alignment_cost(w, scans{1}, scans{2});
%w = fminsearch(cost_fn, [0;0;0;0]);
%w = randomized_gradient_descent([0;0;0;0], cost_fn)



