%% train and save 5 models using regression %%

filename1 = 'datasets/egomotion feb2017/models/muvx_model';
filename2 = 'datasets/egomotion feb2017/models/muvy_model';
filename3 = 'datasets/egomotion feb2017/models/sigmavx_model';
filename4 = 'datasets/egomotion feb2017/models/sigmavy_model';
filename5 = 'datasets/egomotion feb2017/models/sigmavxvy_model';

%train models
[training_label_vector_muvx, training_instance_matrix_muvx] = libsvmread('datasets/egomotion feb2017/training data/muvx.scale');
muvx_model = svmtrain(training_label_vector_muvx, training_instance_matrix_muvx, '-s 4 -h 0');

[training_label_vector_muvy, training_instance_matrix_muvy] = libsvmread('datasets/egomotion feb2017/training data/muvy.scale');
muvy_model = svmtrain(training_label_vector_muvy, training_instance_matrix_muvy, '-s 4 -h 0');

[training_label_vector_sigmavx, training_instance_matrix_sigmavx] = libsvmread('datasets/egomotion feb2017/training data/sigmavx.scale');
sigmavx_model = svmtrain(training_label_vector_sigmavx, training_instance_matrix_sigmavx, '-s 4 -h 0');

[training_label_vector_sigmavy, training_instance_matrix_sigmavy] = libsvmread('datasets/egomotion feb2017/training data/sigmavy.scale');
sigmavy_model = svmtrain(training_label_vector_sigmavy, training_instance_matrix_sigmavy, '-s 4 -h 0');

[training_label_vector_sigmavxvy, training_instance_matrix_sigmavxvy] = libsvmread('datasets/egomotion feb2017/training data/sigmavxvy.scale');
sigmavxvy_model = svmtrain(training_label_vector_sigmavxvy, training_instance_matrix_sigmavxvy, '-s 4 -h 0');

%save models
save(filename1, '-struct', 'muvx_model');
save(filename2, '-struct', 'muvy_model');
save(filename3, '-struct', 'sigmavx_model');
save(filename4, '-struct', 'sigmavy_model');
save(filename5, '-struct', 'sigmavxvy_model');