function plot_scans(scans, hold_on)
%plot_scans(scans)

if nargin < 2
    hold_on = 1;
end

clf;
if hold_on
    hold on;
end
for i=1:length(scans);
    if isempty(scans{i})
        continue;
    end
    c = colormap('jet');
    plot3(scans{i}(:,1), scans{i}(:,2), scans{i}(:,3), '.', ...
          'Color', c(mod(20*(i-1),size(c,1))+1,:));
    if i>1 && ~hold_on
        hold on;
        plot3(scans{i-1}(:,1), scans{i-1}(:,2), scans{i-1}(:,3), '.', ...
              'Color', c(mod(20*(i-2),size(c,1))+1,:));
        hold off;
    end
    axis vis3d;
    axis equal;
    drawnow;
    %input(':');
end
hold off;
