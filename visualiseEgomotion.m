%% visualise egomotion events %%

clc;
clear;

%load data
eventsfilename = '/home/vvasco/dev/Data/iCub/egomotion/egomotion/book_egomotion/3/vision/data.log.txt';
egomotionfilename = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/testing data/book/book_egomotion.txt';
egomotion = importdata(egomotionfilename);
events = importdata(eventsfilename);

%sensors'parameters
width = 304;
height = 240;

%thresh for mahalanobis distance
% thresh = 5;
% egomotion(egomotion(:, 9) > thresh, :) = [];

nevents = 20000;
fixedevents = [];
xcorn = [];
ycorn = [];
vx = [];
vy = [];
mah_dist = [];
mahal_dist = zeros(height, width);
for i = 1 : size(events, 1) % 1780732
   
    tsi = events(i, 2);
    xi = events(i, 4);
    yi = events(i, 5);
    
    j = find(egomotion(:, 2) == tsi, 1);
    if(sum(j))
        xcorn = [xcorn; xi];
        ycorn = [ycorn; yi];
        vx = [vx; egomotion(j, 7)];
        vy = [vy; egomotion(j, 8)];
        mah_dist = [mah_dist; egomotion(j, 9)];
        mahal_dist(yi + 1, xi + 1) = egomotion(j, 9);
    end
        
    fixedevents = [fixedevents; events(i, :)];
    if(size(fixedevents, 1) > nevents)
        fixedevents(1, :) = [];
    end
    
    %visualise
    if(mod(i, nevents) == 0)
        maxdist = max(max(mahal_dist));
        
        xind_motion = xcorn(mah_dist > thresh);
        yind_motion = ycorn(mah_dist > thresh);
        
        drawnow;
        figure(1);
        plot(fixedevents(fixedevents(:, 3) == 0, 4), fixedevents(fixedevents(:, 3) == 0, 5), 'g.'); hold on;
        plot(fixedevents(fixedevents(:, 3) == 1, 4), fixedevents(fixedevents(:, 3) == 1, 5), 'm.');
        plot(xcorn, ycorn, 'b.'); 
        quiver(xcorn, ycorn, vx, vy, 'b'); 
        plot(xind_motion, yind_motion, 'r.', 'markersize', 15); hold off;        
        axis([0 width 0 height]);
        
        figure(2);
        imagesc(mahal_dist);
%         caxis([0 thresh]);
        axis xy;
        colorbar;    
        
        pause(0.1);
        
        xcorn = [];
        ycorn = [];
        vx = [];
        vy = [];
        mah_dist = [];
        mahal_dist = zeros(height, width);
    end    
end