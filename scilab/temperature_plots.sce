// clear all 
xdel(winsid());
clear;

//open data file in matrix format
data = csvRead('/home/andreu/datasets/zyx/gocator_temperature/exp0_mat.txt');
[rows cols] = size(data);

//generate a time axis
time = [1:1:rows];

//plot internal & projector tempereture
fig_h=figure();
fig_h.background = color("white");
plot(time, data(:,1)./1000., time, data(:,2)./1000.);
ah = gca();
//ah.isoview = "on";
ah.x_label.text = "$time [minutes]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$Temp [ºC]$";
ah.y_label.font_size = 4;
ah.grid = [1,1,1];
ah.grid_position = "background";
ah.auto_clear = "off";
ah.auto_scale = "off";
ah.data_bounds = [0 45; 600 55];
plot_colors = ["r";"g"];

