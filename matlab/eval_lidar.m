% eval_lidar
% Author:
%           Kenneth Laws
%           Here Technologies
% Date: 02/15/2018

% select the reference scan 
disp 'Selecting reference scan'
refname = '../Scans/Standard/intensitySet.ascii';
if exist(refname)     
    s = input('use existing Standard/intensity_set.ascii (Y/n)?','s');
    if isempty(s) || s == 'y' || s == 'Y'
        disp(['using ' refname ]);
    else
        error('run test_lidar on a reference standard to collect scan');
    end
else
    error('run test_lidar on a reference standard to collect scan');
end

% plot reference scan
hfig = figure(1); clf
set(hfig,'position',[.65 .250 .30 .70],'units', 'normalized');
subplot(2,1,1), hold on
title('Reference Lidar Intensity')
plot_inten_scan(refname);

% select the scan for unit under test
disp '************************'
disp 'Selecting test scan'
if exist('../lidar_testing/reference_intensity_set.ascii')     
    s = input('use existing test scan (Y/n)','s');
    if isempty(s) || s == 'y' || s == 'Y'
        disp 'using existing test scan';
        fname = '../lidar_testing/reference_intensity_set.ascii';
    else
        error('run test_lidar to collect test scan');
    end
else
    error('run test_lidar to collect test scan');
end
    
% plot the test scan
subplot(2,1,2), hold on
title('Test Lidar Intensity')
I = plot_inten_scan(fname);

% save the scan data
s = input('Save test scan data (Y/n)','s');
if isempty(s) || s == 'y' || s == 'Y'
    [fname,pname] = uiputfile('*.dlm');
    dlmwrite([pname fname],I);
else
    disp 'discarding test scan data'
end

