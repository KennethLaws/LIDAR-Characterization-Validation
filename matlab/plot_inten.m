% plot lidar intensity
%laser_elev = [-30.67 -9.33 -29.33 -8.00 -28.00 ...]
%I = dlmread('./Scans/Standard/intensitySetS1.ascii');
I = dlmread('../Scans/SampleLidar/intensitySet6.ascii');

laser = I(:,1);
angle = I(:,2);
range = I(:,3);
intens = I(:,4);

% figure(1), clf, hold all
% 
% for k = [21 23 27 29 31]
%     idx = find(laser == k+1);
%     ang = angle(idx);
%     rng = range(idx);
%     ints = intens(idx);
%     fprintf('laser number: %d,points collected: %d\n',k,length(idx));
%     plot(ang,ints,'*');
%     xlabel('Angle (deg)');
%     ylabel('Intensity value');
% end

figure(1), clf, hold all
for k = 0:31
    idx = find(laser == k+1);
    ang = angle(idx);
    rng = range(idx);
    ints = intens(idx);
    fprintf('laser number: %d,points collected: %d\n',k,length(idx));
    plot(ang,ints,'*');
    xlabel('Angle (deg)');
    ylabel('Diffuse Reflectance (%)');
    xlim([267.5 279])
    if max(ints)< 200
        ylim([0 200])
    end
end
figure(2), clf, hold all
for k = 0:31
    idx = find(laser == k+1);
    ang = angle(idx);
    rng = range(idx);
    ints = intens(idx);
    fprintf('laser number: %d,points collected: %d\n',k,length(idx));
    plot(ang,rng,'*');
    xlabel('Angle (deg)');
    ylabel('Diffuse Reflectance (%)');
    xlim([267.5 279])
    if max(ints)< 200
        ylim([0 200])
    end

end
