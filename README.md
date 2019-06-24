# Tools for datasets published by the Oxford Estimation, Search, and Planning research group.
Plot Vicon trajectories
```
[vicon_trajectories, vicon_timestamps] = ingest_vicon_data('vicon.csv');
plot_multiple_trajectories(vicon_trajectories);
```

Project Vicon frames into cameara images
```
path_to_kalibr_calibration = [path_to_dataset 'kalibr_2019_06_14.yaml'];
path_to_manufacturer_calibration = [path_to_dataset 'manufacturer_2019_06_14.yaml'];
path_to_vicon_calibration = [path_to_dataset 'vicon_2019_06_14.yaml'];
path_to_images = [path_to_dataset data_segment];
draw_axes_on_image;
```
