% plot lidar intensity
%laser_elev = [-30.67 -9.33 -29.33 -8.00 -28.00 ...]
I = dlmread('./Scans/Standard/intensitySetS1.ascii');
laser = I(:,1);
angle = I(:,2);
range = I(:,3);
intens = I(:,4);

I = dlmread('./Scans/RedSet1/intensitySetR2.ascii');
test_laser = I(:,1);
test_angle = I(:,2);
test_range = I(:,3);
test_intens = I(:,4);



figure(1), clf, hold all
for k = 0:31
    idx = find(laser == k+1);
    ang = angle(idx);
    rng = range(idx);
    ints = intens(idx);
    fprintf('laser number: %d,points collected: %d\n',k,length(idx));
    plot(ang,ints,'*');
    xlabel('Angle (deg)');
    ylabel('Intensity value');
    xlim([267.5 279])
end
text(276,180,'Standard')

figure(2), clf, hold all
for k = 0:31
    idx = find(test_laser == k+1);
    ang = test_angle(idx);
    rng = test_range(idx);
    ints = test_intens(idx);
    fprintf('laser number: %d,points collected: %d\n',k,length(idx));
    plot(ang,ints,'*');
    xlabel('Angle (deg)');
    ylabel('Intensity value');
    xlim([267.5 279])
end

% establish the standard
% find the 100% return angle region
% compute the mean and std over this region, excluding edges and direct
% reflectors
% find the 90% region, excluding edges, find the mean and std over the 32
% lasers
% repeat for 15% region
% for the UUT, for each region, compute: the mean over all lasers, identify
% any lasers that are 2 std out for some percentage of the given region.

