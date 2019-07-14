# Tools for datasets published by the Oxford Estimation, Search, and Planning research group.

## Dependencies
### yamlmatlab
Required to load camera calibrations (https://github.com/ewiger/yamlmatlab).

## Examples
### Plot Vicon trajectories
```
[vicon_trajectories, vicon_timestamps] = ingest_vicon_data('vicon.csv');
plot_multiple_trajectories(vicon_trajectories);
```

### Project Vicon frames into camera images
```
path_to_kalibr_calibration = [path_to_dataset '/kalibr_2019_06_14.yaml'];
path_to_manufacturer_calibration = [path_to_dataset '/manufacturer_2019_06_14.yaml'];
path_to_vicon_calibration = [path_to_dataset '/vicon_2019_06_14.yaml'];
path_to_images = [path_to_dataset data_segment ];
path_to_images = [path_to_dataset '/vicon.csv'];
draw_axes_on_image;
```
