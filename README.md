# VAMR-MP
Monocular Visual Odometry pipeline for the mini project of the course Vision Algorithms for Mobile Robotics, Fall 21-22.

## Authors
This project is done by:<br/>
Cafer Mertcan Akcay, MSc Student in Robotics, Systems and Control<br/>
Irem Kaftan, MSc Student in Information Technology and Electrical Engineering

## Datasets
Three publics datasets (KITTI, Malaga, Parking) and two custom datasets (Market and Minecraft) are used for the project. They are located in datasets folder.

## Requirements
The project is coded in MATLAB 2021b and we ran it on a computer with an Intel i7 (2.80GHz) processor and a RAM of 16GB. On average, it uses 20% of CPU and 2GB RAM. You can run the code just by running 'main.m'. The dataset selection is done by changing 'ds' parameter. It can be also set if the operation will be displayed in a figure or saved in a folder while running by setting 'save_or_display' parameter. By default it displays the output after processing first 20 frames.

## Results
The results of the pipeline and raw videos of custom datasets can be seen in this playlist https://youtube.com/playlist?list=PLYtyCJjambCG9-n2uwoR_akEnG9yzT7Qm. Please note that the videos are created from saved figures to have a video with uniform time (2fps) between frames. However, actual runtime is similar to videos (1-2fps) although it depends on the dataset and it may change from frame to frame.
