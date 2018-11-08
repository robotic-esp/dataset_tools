function [lin_acc, quaternions, imu_timestamps] = ingest_imu_data(filename)





% load vicon data
imu_data = readtable(filename);
lin_acc = zeros(size(imu_data,1),3);
quaternions = zeros(size(imu_data,1),4);

w = waitbar(0, '');
for i=1:size(imu_data,1)
    lin_acc(i,1:3) = imu_data{i,10:12};
    quaternions(i,1:4) = imu_data{i,3:6};
    waitbar(i/size(imu_data,1),w, [num2str(i) '/' num2str(size(imu_data,1)) ' entries read']);
end
close(w);
imu_timestamps = imu_data{1:length(lin_acc):end,1} + imu_data{1:length(lin_acc):end,2}/1e9;

end