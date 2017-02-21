function tracks = initialiseTracks(diameterx, diametery, width, height)
%sample the image at pxcovered rate and initialise the trackers centered at
%pxcovered rate
%tracks = [time centerx centery activity state]

tracksize = width/diameterx * height/diametery;
tracks = cell(1, tracksize);

kk = 1;
while kk < tracksize
    for xx = diameterx/2 : diameterx : width
        for yy = diametery/2 : diametery : height
            tracks{1, kk} = [0 xx yy 0 0 0 0];
            kk = kk + 1;
        end
    end
end

