clear;
clc;
close all

K=[6.1145098876953125e+02, 0,4.3320397949218750e+02;
         0, 6.1148571777343750e+02, 2.4947302246093750e+02;
         0,0,1];
Tbc=[  9.9792816252667338e-03, 6.5348103708624539e-03,...
             9.9992885256485176e-01, 2.2648368490900000e-01;...
             -9.9982014658446139e-01, 1.6192923276330706e-02,...
             9.8723715283343672e-03, -5.1141940356500000e-02;...
             -1.6127257115523985e-02, -9.9984753112121250e-01,...
             6.6952288046080444e-03, 9.1600000000000004e-01; 0., 0., 0.,...
             1. ];

season='office1-7';

datapath='/media/slinkle/ChuChu/jiao/dataset/openloris-scene/data/';
matpath='/media/slinkle/ChuChu/jiao/dataset/openloris-scene/data/mat/';
lesspath='/media/slinkle/GrainFull/jiao/dataset/lessOpenloris/data/';
lessmatpath=[lesspath,'mat/'];

fprintf('loading data ...\n');

season_color = load([matpath,season,'color_time.mat']);
season_color = season_color.Allname;
season_pose = load([matpath,season,'gtpose.mat']);
season_pose = season_pose.Allpose;

pngs=dir([datapath,season,'/color/*.png']);
pngs={pngs.name};
mappngs={};
mapposes={};

R_i=[];
t_i=[];


for i=1:length(pngs)
    disp(['detail: ',num2str(i),' % ',num2str(length(pngs))]);
    map_name=pngs{i};
%     pos=strfind(map_name,'.');
    map_time_str=map_name(1:end-4);
    map_time=str2double(map_time_str);
    
    pos=find(season_pose(:,1)==map_time);
    pose_cw=season_pose(pos,2:8);
    quat_cw=pose_cw(4:7);
    t_cw=pose_cw(1:3).';
    R_cw=quat2rotm(quat_cw);
    T_wb=[R_cw,t_cw; 0 0 0 1];%T_wb
    T_wc=T_wb*Tbc;
    R_wc=T_wc(1:3,1:3);
    t_wc=T_wc(1:3,4);
    
    if isempty(R_i)
        R_i=R_wc;
        t_i=t_wc;
        mappngs=[mappngs,map_name];
        mapposes=[mapposes,T_wc];
        copyfile([datapath,season,'/color/',map_name],[lesspath,season,'/color/',map_name]);
        continue;
    end
    
    delta_t = norm(t_wc-t_i);
    if delta_t>1
        mappngs=[mappngs,map_name];
        mapposes=[mapposes,T_wc];
        copyfile([datapath,season,'/color/',map_name],[lesspath,season,'/color/',map_name]);
        R_i=R_wc;
        t_i=t_wc;
        continue;
    end
    
    quatError=rotm2axang(R_wc/R_i);%degree
    delta_theta=quatError(4);
    delta_theta=delta_theta/pi*180;%degree
    
    if abs(delta_theta)>20
        mappngs=[mappngs,map_name];
        mapposes=[mapposes,T_wc];
        copyfile([datapath,season,'/color/',map_name],[lesspath,season,'/color/',map_name]);
        R_i=R_wc;
        t_i=t_wc;
        continue;
    end
    
    
end

save([lessmatpath,'mappngs.mat'],'mappngs','mapposes');