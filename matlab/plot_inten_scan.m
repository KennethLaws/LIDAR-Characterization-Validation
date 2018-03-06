function I = plot_inten_scan(fname)
% Author:
%           Kenneth Laws
%           Here Technologies
% Date: 02/15/2018


% plot lidar intensity
I = dlmread(fname);

laser = I(:,1);
angle = I(:,2);
range = I(:,3);
intens = I(:,4);

for k = 0:31
    idx = find(laser == k+1);
    ang = angle(idx);
    rng = range(idx);
    ints = intens(idx);
    %fprintf('laser number: %d,points collected: %d\n',k,length(idx));
    plot(ang,ints,'*');
    xlabel('Angle (deg)');
    ylabel('Diffuse Reflectance (%)');
    xlim([267.5 279])
    if max(ints)< 200
        ylim([0 200])
    end
end
return

