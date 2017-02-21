%% compute mahalanobis distance %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BEFORE USING THIS, SCALE THE TESTING DATASET!!!!!%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

%load trained models
display('Loading pre-trained models...');
muvx_model = open('datasets/models/no parameter selection/muvx_model.mat');
muvy_model = open('datasets/models/no parameter selection/muvy_model.mat');
sigmavx_model = open('datasets/models/no parameter selection/sigmavx_model.mat');
sigmavy_model = open('datasets/models/no parameter selection/sigmavy_model.mat');
sigmavxvy_model = open('datasets/models/no parameter selection/sigmavxvy_model.mat');

%load data
display('Loading data...');
muvx_file = '/home/vvasco/dev/egomotion/datasets/training data/muvx-ae-test.scale';
muvy_file = '/home/vvasco/dev/egomotion/datasets/training data/muvy-ae-test.scale';
sigmavx_file = '/home/vvasco/dev/egomotion/datasets/training data/sigmavx-ae-test.scale';
sigmavy_file = '/home/vvasco/dev/egomotion/datasets/training data/sigmavy-ae-test.scale';
sigmavxvy_file = '/home/vvasco/dev/egomotion/datasets/training data/sigmavxvy-ae-test.scale';
muvx = dlmread(muvx_file);
muvy = dlmread(muvy_file);
sigmavx = dlmread(sigmavx_file);
sigmavy = dlmread(sigmavy_file);
sigmavxvy = dlmread(sigmavxvy_file);

tencfile = '//home/vvasco/dev/egomotion/datasets/training data/timeencoders_xae.txt';
timeencoders = importdata(tencfile);

flowname = '/home/vvasco/dev/egomotion/datasets/training data/flow/flow_xae.txt';
flow = importdata(flowname);

%result
outfilename = '/home/vvasco/dev/egomotion/datasets/training data/egomotion_xae.txt';

muvx = [timeencoders muvx(:, 1) muvx(:, 3) muvx(:, 5) muvx(:, 7) muvx(:, 9) muvx(:, 11) muvx(:, 13)];
muvy = [timeencoders muvy(:, 1) muvy(:, 3) muvy(:, 5) muvy(:, 7) muvy(:, 9) muvy(:, 11) muvy(:, 13)];
sigmavx = [timeencoders sigmavx(:, 1) sigmavx(:, 3) sigmavx(:, 5) sigmavx(:, 7) sigmavx(:, 9) sigmavx(:, 11) sigmavx(:, 13)];
sigmavy = [timeencoders sigmavy(:, 1) sigmavy(:, 3) sigmavy(:, 5) sigmavy(:, 7) sigmavy(:, 9) sigmavy(:, 11) sigmavy(:, 13)];
sigmavxvy = [timeencoders sigmavxvy(:, 1) sigmavxvy(:, 3) sigmavxvy(:, 5) sigmavxvy(:, 7) sigmavxvy(:, 9) sigmavxvy(:, 11) sigmavxvy(:, 13)];

ienc = 1;
perc = 10;
% x = [];
% y = [];
% ts = [];
predvx = [];
predvy = [];
v = [];
display('Computing Mahalanobis distance...');
%for each flow event 
for ii = 1 : size(flow, 1)
   
    %current velocity
    tsi = flow(ii, 2);
    xi = flow(ii, 4);
    yi = flow(ii, 5);
    vx = flow(ii, 7);
    vy = flow(ii, 8);
    tbottle = flow(ii, 6);
%     x = [x xi];
%     y = [y yi];
%     ts = [ts tsi];
    v = [v [vx; vy]];
    
    %when we receive the encoder value
    tenc = muvx(ienc, 1);
    if(tbottle > tenc)

        %we use the encoder values
        testing_instance_matrix = muvx(ienc, 3:end);
        testing_label = rand;
        
        %to predict optical flow statistics using the learnt models
        [predicted_muvx] = svmpredict(testing_label, testing_instance_matrix, muvx_model);
        [predicted_muvy] = svmpredict(testing_label, testing_instance_matrix, muvy_model);
        [predicted_sigmavx] = svmpredict(testing_label, testing_instance_matrix, sigmavx_model);
        [predicted_sigmavy] = svmpredict(testing_label, testing_instance_matrix, sigmavy_model);
        [predicted_sigmavxvy] = svmpredict(testing_label, testing_instance_matrix, sigmavxvy_model);
        
        predvx(ienc) = predicted_muvx;
        predvy(ienc) = predicted_muvy;
        
        %predicted statistics
        muv = [predicted_muvx; predicted_muvy];
        sigmav = [predicted_sigmavx predicted_sigmavxvy;
            predicted_sigmavxvy predicted_sigmavy];
                
        mah_dist = zeros(size(v, 2), 1);
        for kk = 1 : size(v, 2)
            
            delta = v(:, kk) - muv;
            [mat, posdef] = chol(sigmav);
            
            %if sigma is not positive-definite, compute the nearest
            %positive-definite matrix
            if(posdef ~= 0)
                mat = nearestSPD(sigmav);
            end
            
            %compute Mahalanobis distance
            z = mat\delta;
            md = z'*z;
            mah_dist(kk) = sqrt(md);              
                        
            %if Mahalanobis distance > threshold
            
                %classify corner event as egomotion event
            
            %else
            
                %only egomotion
                
            flow(ii - size(v, 2) + kk, 9) = mah_dist(kk);
        end
        
        ienc = ienc + 1;
        if(ienc > length(timeencoders))
            break;
        end
        
%         x = [];
%         y = [];
%         ts = [];
        v = [];
    end
    
    %display percentage
    if( ( ( ii / size(flow, 1) ) * 100 ) > perc )
        display([int2str( (ii / size(flow, 1)) * 100 ) '% done']);
        perc = perc + 10;
    end    
end

dlmwrite(outfilename, flow, 'delimiter', ' ', 'precision', 20);