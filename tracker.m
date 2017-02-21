%% feature tracker %% 
% Based on "Asynchronous Event-Based Multikernel Algorithm for High Speed 
% Visual Features Tracking" 

clear;

filename = '/home/vvasco/dev/egomotion/datasets/testing data/book/corners_book.txt';
corners = importdata(filename);

%resultfile 
outfilename = '/home/vvasco/dev/egomotion/datasets/testing data/book/flow_book.txt';

%sensor parameters
width = 304;
height = 240;

%trackers'size
trackerdiameterx = 16;
trackerdiametery = 16;

%parameters
tau = 5000000; % 50000000;
tAct = 0.005; % 0.5
tInact = 0.001; % 0.1;
tFree = 0.0005; % 0.1;
alpha_pos = 0.1;
timewindow = 100000; % 1000000;
timescale = 1000000;

%set of initial trackers
tracker_pool_on = initialiseTracks(trackerdiameterx, trackerdiametery, width, height);
tracker_pool_off = initialiseTracks(trackerdiameterx, trackerdiametery, width, height);

% figure(1); hold on;
% for rr = 1 : size(tracker_pool_on, 2)
%     rectangle('position', [tracker_pool_on{1, rr}(end, 2)-trackerdiameterx/2 ...
%         tracker_pool_on{1, rr}(end, 3)-trackerdiametery/2 ...
%         trackerdiameterx trackerdiametery], 'curvature', [1 1]);
%     text(tracker_pool_on{1, rr}(end, 2), tracker_pool_on{1, rr}(end, 3), num2str(rr));
% end
% axis([0 width+trackerdiameterx 0 height+trackerdiametery]);
% hold off;

%extend corner events for velocity
corners = [corners zeros(size(corners, 1), 2)];

perc = 10;
%for each corner event
for ii = 1 : size(corners, 1)

    %current event
    xi = corners(ii, 4);
    yi = corners(ii, 5);
    ti = corners(ii, 2);
    poli = corners(ii, 3);
    
    if(poli == 0)        
        %look for the tracker with the biggest p
        p = zeros(size(tracker_pool_on, 2), 1);
        for jj_on = 1 : size(tracker_pool_on, 2)
            tracker_sz = size(tracker_pool_on{1, jj_on}, 1);
            cx = tracker_pool_on{1, jj_on}(tracker_sz, 2);
            cy = tracker_pool_on{1, jj_on}(tracker_sz, 3);
            p(jj_on) = compute_p(cx, cy, xi, yi, 1, 1, 0);
        end
        [maxp, id_track] = max(p);
        
        %update the tracker with the highest probability
        sel_tracker_sz = size(tracker_pool_on{1, id_track}, 1);
        cxsel = tracker_pool_on{1, id_track}(sel_tracker_sz, 2);
        cysel = tracker_pool_on{1, id_track}(sel_tracker_sz, 3);
        tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 1) = ti;
        tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 2) = (1-alpha_pos)*cxsel + alpha_pos*xi;
        tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 3) = (1-alpha_pos)*cysel + alpha_pos*yi;
        tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 4) = tracker_pool_on{1, id_track}(sel_tracker_sz, 4) + maxp; %%%% 1;
        
        %if A > tAct, upgrade
        if(tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 4) > tAct)
            tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 5) = 1;
        else
            tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 5) = 0;
        end
        
        %decay the activity of each tracker
        for ww_on = 1 : size(tracker_pool_on, 2)
            if(ww_on ~= id_track)
                tracker_sz = size(tracker_pool_on{1, ww_on}, 1);
                dt = ti - tracker_pool_on{1, ww_on}(tracker_sz, 1);
                tracker_pool_on{1, ww_on}(tracker_sz, 4) = tracker_pool_on{1, ww_on}(tracker_sz, 4) * exp(-dt/tau);
                
                %if A < tInact, downgrade
                if(tracker_pool_on{1, ww_on}(tracker_sz, 4) < tInact)
                    tracker_pool_on{1, ww_on}(tracker_sz, 5) = 0;
                else if (tracker_pool_on{1, ww_on}(tracker_sz, 4) > tInact)
                        tracker_pool_on{1, ww_on}(tracker_sz, 5) = 1;
                        
                        %if A < tFree, destroy
                    else if (tracker_pool_on{1, ww_on}(tracker_sz, 4) < tFree)
                            tracker_pool_on{1, ww_on} = [];
                        end
                    end
                end
            end
        end
        
        if(tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 5) == 1)
            vx = (tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 2) - tracker_pool_on{1, id_track}(sel_tracker_sz, 2)) / ...
                (tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 1) - tracker_pool_on{1, id_track}(sel_tracker_sz, 1));
            vy = (tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 3) - tracker_pool_on{1, id_track}(sel_tracker_sz, 3)) / ...
                (tracker_pool_on{1, id_track}(sel_tracker_sz + 1, 1) - tracker_pool_on{1, id_track}(sel_tracker_sz, 1));
        else
            vx = 0;
            vy = 0;
        end
    else
        %look for the tracker with the biggest p
        p = zeros(size(tracker_pool_off, 2), 1);
        for jj_off = 1 : size(tracker_pool_off, 2)
            tracker_sz = size(tracker_pool_off{1, jj_off}, 1);
            cx = tracker_pool_off{1, jj_off}(tracker_sz, 2);
            cy = tracker_pool_off{1, jj_off}(tracker_sz, 3);
            p(jj_off) = compute_p(cx, cy, xi, yi, 1, 1, 0);
        end
        [maxp, id_track] = max(p);
        
        %update the tracker with the highest probability
        sel_tracker_sz = size(tracker_pool_off{1, id_track}, 1);
        cxsel = tracker_pool_off{1, id_track}(sel_tracker_sz, 2);
        cysel = tracker_pool_off{1, id_track}(sel_tracker_sz, 3);
        tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 1) = ti;
        tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 2) = (1-alpha_pos)*cxsel + alpha_pos*xi;
        tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 3) = (1-alpha_pos)*cysel + alpha_pos*yi;
        tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 4) = tracker_pool_off{1, id_track}(sel_tracker_sz, 4) + maxp; %%%% 1;
        
        %if A > tAct, upgrade
        if(tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 4) > tAct)
            tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 5) = 1;
        else
            tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 5) = 0;
        end
        
        %decay the activity of each tracker
        for ww_off = 1 : size(tracker_pool_off, 2)
            if(ww_off ~= id_track)
                tracker_sz = size(tracker_pool_off{1, ww_off}, 1);
                dt = ti - tracker_pool_off{1, ww_off}(tracker_sz, 1);
                tracker_pool_off{1, ww_off}(tracker_sz, 4) = tracker_pool_off{1, ww_off}(tracker_sz, 4) * exp(-dt/tau);
                
                %if A < tInact, downgrade
                if(tracker_pool_off{1, ww_off}(tracker_sz, 4) < tInact)
                    tracker_pool_off{1, ww_off}(tracker_sz, 5) = 0;
                else if (tracker_pool_off{1, ww_off}(tracker_sz, 4) > tInact)
                        tracker_pool_off{1, ww_off}(tracker_sz, 5) = 1;
                        
                        %if A < tFree, destroy
                    else if (tracker_pool_off{1, ww_off}(tracker_sz, 4) < tFree)
                            tracker_pool_off{1, ww_off} = [];
                        end
                    end
                end
            end
        end
        
        if(tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 5) == 1)
            vx = (tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 2) - tracker_pool_off{1, id_track}(sel_tracker_sz, 2)) / ...
                (tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 1) - tracker_pool_off{1, id_track}(sel_tracker_sz, 1));
            vy = (tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 3) - tracker_pool_off{1, id_track}(sel_tracker_sz, 3)) / ...
                (tracker_pool_off{1, id_track}(sel_tracker_sz + 1, 1) - tracker_pool_off{1, id_track}(sel_tracker_sz, 1));
        else
            vx = 0;
            vy = 0;
        end
    end
    
    %associate to the corner the velocity of its cluster
    corners(ii, 7) = vx*timescale;
    corners(ii, 8) = vy*timescale;
    
    %display percentage
    if( ( ( ii / size(corners, 1) ) * 100 ) > perc )
        display([int2str( (ii / size(corners, 1)) * 100 ) '% done']);
        perc = perc + 10;
    end
        
%     %visualise
%     if(mod(ii, 2000) == 0)
%         drawnow;
%         figure(2); 
%         plot(corners(ii - 999 : ii, 4), corners(ii - 999 : ii, 5), 'b.', 'markersize', 10); hold on; 
%         quiver(corners(ii - 999 : ii, 4), corners(ii - 999 : ii, 5), ...
%             corners(ii - 999 : ii, 7), corners(ii - 999 : ii, 8), 'r'); 
%         for rr = 1 : size(tracker_pool_on, 2)
%             %inactive trackers
%             if(tracker_pool_on{1, rr}(end, 5) == 0)
%                 %                 rectangle('position', [tracker_pool_on{1, rr}(end, 2)-trackerdiameterx/2 ...
%                 %                     tracker_pool_on{1, rr}(end, 3)-trackerdiametery/2 ...
%                 %                     trackerdiameterx trackerdiametery], 'curvature', [1 1], 'edgecolor', 'w');
%                 
%                 %active trackers
%             else
%                 rectangle('position', [tracker_pool_on{1, rr}(end, 2)-trackerdiameterx/2 ...
%                     tracker_pool_on{1, rr}(end, 3)-trackerdiametery/2 ...
%                     trackerdiameterx trackerdiametery], 'curvature', [1 1], 'edgecolor', 'g');
%             end
%         end
%         for oo = 1 : size(tracker_pool_off, 2)
%             %inactive trackers
%             if(tracker_pool_off{1, oo}(end, 5) == 0)
%                 %                 rectangle('position', [tracker_pool_off{1, oo}(end, 2)-trackerdiameterx/2 ...
%                 %                     tracker_pool_off{1, oo}(end, 3)-trackerdiametery/2 ...
%                 %                     trackerdiameterx trackerdiametery], 'curvature', [1 1], 'edgecolor', 'w');
%                 
%                 %active trackers
%             else
%                 rectangle('position', [tracker_pool_off{1, oo}(end, 2)-trackerdiameterx/2 ...
%                     tracker_pool_off{1, oo}(end, 3)-trackerdiametery/2 ...
%                     trackerdiameterx trackerdiametery], 'curvature', [1 1], 'edgecolor', 'm');
%             end
%         end
%         axis([0 width+trackerdiameterx 0 height+trackerdiametery]);
%         hold off;
%     end
    
end

dlmwrite(outfilename, corners, 'delimiter', ' ', 'precision', 20);
