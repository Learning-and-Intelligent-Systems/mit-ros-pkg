cnt = 10;
G_tot = [];
for i=0:cnt-1
    run(sprintf('out%d',i));
    F_dims = F_dims([4,2,1]);
    G = reshape(F, F_dims);
    G_tot(i*F_dims(1)+1:(i+1)*F_dims(1),:,:) = (G-.6)/4;
end
for j=1:min(size(G_tot,2),size(G_tot,3))
    figure(1);
    subplot(2,1,1);
    imshow(reshape(G_tot(:,j,:), [size(G_tot,1),size(G_tot,3)])');
    colormap hot;
    title(sprintf('y = %d',j));
    xlabel('theta');
    ylabel('x');
    subplot(2,1,2);
    imshow(reshape(G_tot(:,:,j), [size(G_tot,1),size(G_tot,2)])');
    colormap hot;
    title(sprintf('x = %d',j));
    xlabel('theta');
    ylabel('y');
    pause(.5);
end
if size(G_tot,2) > size(G_tot,3)
    for j=size(G_tot,3)+1:size(G_tot,2)
        figure(1);
        subplot(2,1,1);
        imshow(reshape(G_tot(:,j,:), [size(G_tot,1),size(G_tot,3)])');
        colormap hot;
        title(sprintf('y = %d',j));
        xlabel('theta');
        ylabel('x');
        pause(.5);
    end
elseif size(G_tot,3) > size(G_tot,2)
    for j=size(G_tot,3)+1:size(G_tot,2)
        figure(1);
        subplot(2,1,2);
        imshow(reshape(G_tot(:,:,j), [size(G_tot,1),size(G_tot,2)])');
        colormap hot;
        title(sprintf('x = %d',j));
        xlabel('theta');
        ylabel('y');
        pause(.5);
    end
end