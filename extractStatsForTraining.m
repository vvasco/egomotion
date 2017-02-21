%% extract flow statistics %%

clear;

%output files
muvx_file = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/muvx';
muvy_file = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/muvy';
sigmavx_file = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/sigmavx';
sigmavy_file = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/sigmavy';
sigmavxvy_file = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/sigmavxvy';

fid1 = fopen(muvx_file, 'wt');
fid2 = fopen(muvy_file, 'wt');
fid3 = fopen(sigmavx_file, 'wt');
fid4 = fopen(sigmavy_file, 'wt');
fid5 = fopen(sigmavxvy_file, 'wt');

%load data
jointvelname = '/home/vvasco/dev/Data/iCub/egomotion_feb2017/pure_egomotion/encoders/data.log';
[t, v0, v1, v2, v3, v4, v5] = joint_velocities(jointvelname);
encoders = [v0, v1, v2, v3, v4, v5];

flowfilename = '/home/vvasco/dev/matlab/testing/egomotion/libsvm-3.20/matlab/datasets/egomotion feb2017/training data/flow/flow_egomotion.txt';
flow = importdata(flowfilename);

%sensor's parameters
width = 304;
height = 250;

%remove one channel
% flow(flow(:, 1) == 0, :) = [];

% icut_start = find(t > events(1, 6), 1);
% icut_end = find(t > events(end, 6), 1);
% t = t(icut_start : icut_end);
% v0 = v0(icut_start : icut_end);
% v1 = v1(icut_start : icut_end);
% v2 = v2(icut_start : icut_end);
% v3 = v3(icut_start : icut_end);
% v4 = v4(icut_start : icut_end);
% v5 = v5(icut_start : icut_end);

%initialise data structure
ienc = 1;
vx = [];
vy = [];
muvx = zeros(length(t), 1);
muvy = zeros(length(t), 1);
sigmavx = zeros(length(t), 1);
sigmavy = zeros(length(t), 1);
sigmavxvy = zeros(length(t), 1);
perc = 10;

%for each event
for ii = 1 : size(flow, 1)
    
    %current event
    tbottle = flow(ii, 6);
    velx = flow(ii, 7);
    vely = flow(ii, 8);
    
    vx = [vx; velx];
    vy = [vy; vely];    
       
    %when we receive the encoder value
    if(tbottle > t(ienc))
        muvx(ienc) = mean(vx(1:end-1)); %(don't include the current event because it belongs to the next bottle)
        muvy(ienc) = mean(vy(1:end-1));
        cov_mat = cov(vx(1:end-1), vy(1:end-1));
        if(size(cov_mat, 1) > 1)
            sigmavx(ienc) = cov_mat(1, 1);
            sigmavy(ienc) = cov_mat(2, 2);
            sigmavxvy(ienc) = cov_mat(1, 2);
        else
            sigmavx(ienc) = 0;
            sigmavy(ienc) = 0;
            sigmavxvy(ienc) = 0;
        end
        ienc = ienc + 1;
        vx = velx;
        vy = vely;
    end
        
    %display percentage
    if( ( ( ii / size(flow, 1) ) * 100 ) > perc )
        display([int2str( (ii / size(flow, 1)) * 100 ) '% done']);
        perc = perc + 10;
    end
end    

muvxout = [muvx v0 v1 v2 v3 v4 v5];
muvyout = [muvy v0 v1 v2 v3 v4 v5];
sigmavxout = [sigmavx v0 v1 v2 v3 v4 v5];
sigmavyout = [sigmavy v0 v1 v2 v3 v4 v5];
sigmavxvyout = [sigmavxvy v0 v1 v2 v3 v4 v5];

%convert to svm format: label 1:value 2:value ecc...
for k = 1 : length(v0)     
    c1 = {'1' num2str(v0(k))}; c1 =  strjoin(c1, ':');
    c2 = {'2' num2str(v1(k))}; c2 =  strjoin(c2, ':');
    c3 = {'3' num2str(v2(k))}; c3 =  strjoin(c3, ':');
    c4 = {'4' num2str(v3(k))}; c4 =  strjoin(c4, ':');
    c5 = {'5' num2str(v4(k))}; c5 =  strjoin(c5, ':');
    c6 = {'6' num2str(v5(k))}; c6 =  strjoin(c6, ':');   
    mx = [num2str(muvxout(k)) ' ' c1 ' ' c2 ' ' c3 ' ' c4 ' ' c5 ' ' c6];
    my = [num2str(muvyout(k)) ' ' c1 ' ' c2 ' ' c3 ' ' c4 ' ' c5 ' ' c6];
    sx = [num2str(sigmavxout(k)) ' ' c1 ' ' c2 ' ' c3 ' ' c4 ' ' c5 ' ' c6];
    sy = [num2str(sigmavyout(k)) ' ' c1 ' ' c2 ' ' c3 ' ' c4 ' ' c5 ' ' c6];
    sxy = [num2str(sigmavxvyout(k)) ' ' c1 ' ' c2 ' ' c3 ' ' c4 ' ' c5 ' ' c6];
    
    %write to file
    fprintf(fid1, '%s\n', mx);
    fprintf(fid2, '%s\n', my);
    fprintf(fid3, '%s\n', sx);
    fprintf(fid4, '%s\n', sy);
    fprintf(fid5, '%s\n', sxy);
end
