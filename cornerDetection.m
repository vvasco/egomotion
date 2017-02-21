%% creating the harris scores %%

clc;
%close all;
clear;

%input file
filename = '/home/vvasco/dev/Data/iCub/egomotion/book_egomotion/3/vision/data.log.txt';

%output file
resultfile = '/home/vvasco/datasets/testing data/book/corners_book.txt';
fid = fopen(resultfile, 'wt');

%sensor's parameters
width = 304;
height = 240;

%number of events to process
N = 5000;
minevts = 50;

%size of the spatial window
L = 7; 
filterSize = 7; % always filterSize < L

%threshold for corners
thresh = 2;

%load in events
events = importdata(filename);

%remove unneeded events (polarity etc)
% events(events(:, 1) ~= 0, :) = [];
% events(events(:, 3) ~= 1, :) = [];

%convert to seconds
% events(:, 2) = (events(:, 2) - events(1, 2))./1000000;

%initialise data structures
vSurfON = zeros(height, width);
vSurfOFF = zeros(height, width);
harrison = zeros(height, width);
harrisoff = zeros(height, width);
% score = zeros(size(events, 1), 1);

%Sobel Kernels
for i = 1 : filterSize
    Sx(i) = factorial((filterSize - 1))/((factorial((filterSize - 1) - (i - 1)))*(factorial(i - 1)));
    Dx(i) = Pasc(i - 1, filterSize - 2) - Pasc(i - 2, filterSize - 2);
end
Sy = Sx';
Dy = Dx';
Gx = Sy(:)*Dx;
Gy = Gx';

%gaussian kernel
sigma = 1;
A = 1/(2*pi*sigma^2);
[xw, yw] = meshgrid(-round((L - 1)/2):round((L - 1)/2), -round((L - 1)/2):round((L - 1)/2)); 
h = A * exp(-(xw.^2 + yw.^2) / (2*sigma^2));
h = h/sum(sum(h));

perc = 10;
for i = 1:size(events, 1)

    xi = events(i, 4);
    yi = events(i, 5);
    tsi = events(i, 2);
    poli = events(i, 3);
    
    %process differently for different polarities 
    if(poli == 0) %if ON event
        
        %update the surface
        vSurfON(yi + 1, xi + 1) = tsi;
        
        %if it contains more than N events, remove the oldest
        if(sum(sum(vSurfON > 0)) > N)
            vSurfON(vSurfON == min(vSurfON(vSurfON ~= 0))) = 0;
        end
        
        %the computation is not done on these events, but they are still
        %added to the surface and thus used to detect corner events
        if(xi <= L || xi >= width - L || yi <= L || yi >= height - L)
            continue;
        end
        
        %PROCESS THE HARRIS ON THIS LOCATION
        windEvts = vSurfON(yi + 1 - L : yi + 1 + L, xi + 1 - L : xi + 1 + L) > 0;
        if(sum(sum(windEvts)) > minevts)
            
            dx = zeros(L); dy = dx;
            %apply filter
            for y = 1:filterSize
                for x = 1:filterSize
                    w = windEvts(y : y + filterSize - 1, x : x + filterSize - 1);
                    dx(y, x) = sparseConv(w, Gx);
                    dy(y, x) = sparseConv(w, Gy);
                end
            end
            
            %square derivatives
            dx2 = dx.^2;
            dy2 = dy.^2;
            dxy = dx.*dy;
            
            %         %apply gaussian
            %         dx2 = conv2(dx2, h, 'same');
            %         dy2 = conv2(dy2, h, 'same');
            %         dxy = conv2(dxy, h, 'same');
            
            %create harris matrix
            a = sum(sum(dx2));
            d = sum(sum(dy2));
            b = sum(sum(dxy));
            c = b;
            
            M = [a b;
                c d];
            score = (det(M) - 0.04*(trace(M) ^ 2));
        else
            score = 0;
        end
        harrison(yi + 1, xi + 1) = score;
                
%         if(i > 2*N)
%             evtsubset = events((i-2*N+1):i, :);
%             
%             figure(1);
%             plot(evtsubset(evtsubset(:, 3) == 0, 4), evtsubset(evtsubset(:, 3) == 0, 5), 'g.'); hold on;
%             plot(xi, yi, 'rx', 'markersize', 7, 'linewidth', 3); hold off;
%             axis([0 width 0 height]);
%             
%             figure(2);
%             plot(evtsubset(evtsubset(:, 3) == 1, 4), evtsubset(evtsubset(:, 3) == 1, 5), 'm.'); 
%             axis([0 width 0 height]);
%                         
%             figure(3);
%             imagesc(windEvts);
%             axis xy;
%             colormap(gray);
% %             title(num2str(score));
% 
%             figure(4);
%             subplot(2, 1, 1);
%             histogram(dx, 'binwidth', 1);
%             set(gca, 'xlim', [0 2*L+1]);
%             title(num2str(mode(dx)));
%             subplot(2, 1, 2);
%             histogram(dy, 'binwidth', 1);
%             set(gca, 'xlim', [0 2*L+1]);
%             title(num2str(mode(dy)));
%         end
        
    else %if OFF event
        
        %update the surface
        vSurfOFF(yi + 1, xi + 1) = tsi;
        
        %if it contains more than N events, remove the oldest
        if(sum(sum(vSurfOFF > 0)) > N)
            vSurfOFF(vSurfOFF == min(vSurfOFF(vSurfOFF ~= 0))) = 0;
        end
        
        %the computation is not done on these events, but they are still
        %added to the surface and thus used to detect corner events
        if(xi <= L || xi >= width - L || yi <= L || yi >= height - L)
            continue;
        end
        
        %PROCESS THE HARRIS ON THIS LOCATION
        windEvts = vSurfOFF(yi + 1 - L : yi + 1 + L, xi + 1 - L : xi + 1 + L) > 0;

        if(sum(sum(windEvts)) > minevts)
            
            dx = zeros(L); dy = dx;
            %apply filter
            for y = 1:filterSize
                for x = 1:filterSize
                    w = windEvts(y : y + filterSize - 1, x : x + filterSize - 1);
                    dx(y, x) = sparseConv(w, Gx);
                    dy(y, x) = sparseConv(w, Gy);
                end
            end
            
            %square derivatives
            dx2 = dx.^2;
            dy2 = dy.^2;
            dxy = dx.*dy;
            
            %         %apply gaussian
            %         dx2 = conv2(dx2, h, 'same');
            %         dy2 = conv2(dy2, h, 'same');
            %         dxy = conv2(dxy, h, 'same');
            
            %create harris matrix
            a = sum(sum(dx2));
            d = sum(sum(dy2));
            b = sum(sum(dxy));
            c = b;
            
            M = [a b;
                c d];
            score = (det(M) - 0.04*(trace(M) ^ 2));
        else
            score = 0;
        end
        harrisoff(yi + 1, xi + 1) = score;
    end
    
    
%     if(i > 2*N)
%         if(score > thresh)
%             
%             drawnow;
%             evtsubset = events((i-2*N+1):i, :);
%             
%             figure(1); set(gcf, 'position', [1986           1         927         973]);
%             subplot(2, 1, 1)
%             plot(evtsubset(evtsubset(:, 3) == 0, 4), evtsubset(evtsubset(:, 3) == 0, 5), 'g.'); hold on;
%             if(poli == 0)
%                 plot(xi, yi, 'rx', 'markersize', 7, 'linewidth', 3); hold off;
%             end
%             subplot(2, 1, 2);
%             plot(evtsubset(evtsubset(:, 3) == 1, 4), evtsubset(evtsubset(:, 3) == 1, 5), 'm.'); hold on;
%             if(poli == 1)
%                 plot(xi, yi, 'rx', 'markersize', 7, 'linewidth', 3); hold off;
%             end
%             axis([0 width 0 height]);
%             
%             figure(2);
%             imagesc(windEvts);
%             axis xy;
%             colormap(gray);
%             title(num2str(score));
%             
%             pause(1);
%         end
%     end
    
%     %visualise
%     if(mod(i, 2*N) == 0)
%         drawnow;
%         figure(1); set(gcf, 'position', [993     1   927   973]);        
%         subplot(2, 1, 1);
%         imagesc(vSurfON); 
%         colormap(gray); 
% %         hold on;
% %         contour(harrison, 'r'); hold off;
%         title(num2str( sum(sum(vSurfON > 0)) ));
%         axis xy;
%         subplot(2, 1, 2);
%         imagesc(vSurfOFF); 
%         colormap(gray); 
% %         hold on;
% %         contour(harrisoff, 'r'); hold off;
%         title(num2str( sum(sum(vSurfOFF > 0)) ));
%         axis xy;       
%                 
% %         [xxon, yyon] = find(vSurfON > 0);
% %         evtson = [];
% %         for ion = 1 : length(xxon)
% %             evtson = [evtson; 1 vSurfON(yyon(ion), xxon(ion)) 0 yyon(ion) xxon(ion) harrison(yyon(ion),xxon(ion))];
% %         end
% %         
% %         [xxoff, yyoff] = find(vSurfOFF > 0);
% %         evtsoff = [];
% %         for ioff = 1 : length(xxoff)
% %             evtsoff = [evtsoff; 1 vSurfOFF(yyoff(ioff), xxoff(ioff)) 1 yyoff(ioff) xxoff(ioff) harrisoff(yyoff(ioff), xxoff(ioff))];
% %         end
% %         
% %         cornon = evtson(evtson(:, 6) > thresh, :);
% %         cornoff = evtsoff(evtsoff(:, 6) > thresh, :);
% %         
% %         figure(1); set(gcf, 'position', [993     1   927   973]);
% %         subplot(2, 1, 1)
% %         plot(evtson(:, 4), evtson(:, 5), 'g.'); hold on;
% %         plot(cornon(:, 4), cornon(:, 5), 'r.', 'markersize', 10); hold off;
% %         axis([0 width 0 height]);
% %         subplot(2, 1, 2);
% %         plot(evtsoff(:, 4), evtsoff(:, 5), 'm.'); hold on;
% %         plot(cornoff(:, 4), cornoff(:, 5), 'b.', 'markersize', 10); hold off;
% %         axis([0 width 0 height]);
% %         
%         figure(2); set(gcf, 'position', [1986           1         927         973]);
%         subplot(2, 1, 1);
%         imagesc(harrison);
%         axis xy;
%         colorbar;
%         caxis([thresh 8]);
%         axis([0 width 0 height]);
%         subplot(2, 1, 2);
%         imagesc(harrisoff);
%         axis xy;
%         colorbar;
%         caxis([thresh 8]);
%         axis([0 width 0 height]);
%         
% %         figure(2);
% %         plot(evtsubset(evtsubset(:, 3) == 0, 4), evtsubset(evtsubset(:, 3) == 0, 5), 'g.'); hold on;
% %         plot(evtsubset(evtsubset(:, 3) == 1, 4), evtsubset(evtsubset(:, 3) == 1, 5), 'm.');
% %         plot(cornsubset(:, 4), cornsubset(:, 5), 'r.', 'markersize', 10); hold off;
% %         axis([0 width 0 height]);
%         
% %         harrison = zeros(height, width);
% %         harrisoff = zeros(height, width);
%     end
    
%     if(score > thresh)
%         drawnow;
%         imagesc(windEvts);
%         colormap(gray);
%         axis xy;
%         title([num2str(sum(sum(windEvts))) '    ' num2str(score)]);
%         pause(1);
%     end

    %display percentage
    if( ( ( i / size(events, 1) ) * 100 ) > perc )
        display([int2str( (i / size(events, 1)) * 100 ) '% done']);
        perc = perc + 10;
    end
      
    %attach Harris score to this event in an array
    events(i, 7) = score;
         
end

%save only corners
corners = events(events(:, 7) > thresh, :);
dlmwrite(outfilename, corners, 'delimiter', ' ', 'precision', 20);
% dlmwrite(resultfile, corners, 'delimiter', ' ', 'precision', '%0.6f');
