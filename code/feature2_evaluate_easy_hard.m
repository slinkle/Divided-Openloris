% find nearest point
clear;
clc;

addpath('../');
% addpath('D:\mexopencv-2.4');
addpath('/home/yanmei/mexopencv');


% set the camera intrinsic 904.9990358306457 904.9990358306457 686.1872787475586 498.9065246582031
K=[6.1145098876953125e+02, 0,4.3320397949218750e+02;
         0, 6.1148571777343750e+02, 2.4947302246093750e+02;
         0,0,1];

Tbc=[ 9.9792816252667338e-03, 6.5348103708624539e-03,...
             9.9992885256485176e-01, 2.2648368490900000e-01;...
             -9.9982014658446139e-01, 1.6192923276330706e-02,...
             9.8723715283343672e-03, -5.1141940356500000e-02;...
             -1.6127257115523985e-02, -9.9984753112121250e-01,...
             6.6952288046080444e-03, 9.1600000000000004e-01; 0., 0., 0.,...
             1. ];
         

% seasons={'home1-1','home1-2','home1-3','home1-4','home1-5'};
% seasons={'cafe1-2','cafe1-1'};
% seasons={'corridor1-1','corridor1-2','corridor1-3','corridor1-4','corridor1-5'};
seasons={'office1-1','office1-2','office1-3','office1-4','office1-5','office1-7'};
% methods={'sift','superpoint','r2d2','oritiltr2d2'};


% method='sift';
% method='superpoint';
% method='r2d2';
% method='oritiltr2d2';

for mapsi=1:length(seasons)
    for si=mapsi:length(seasons)
        season=seasons{si};
        mapsession=seasons{mapsi};
        mapseason=mapsession;
        
            opdof=6;
            % opdof=4;
            % topNum=10;
            % topNum=10;
            topNum=1;

            suc_tran_th = 0.5;
            suc_angle_th = 5;

            matpath='/home/yanmei/Data/openloris/market-mat/';
            lessmatpath='/home/yanmei/Data/openloris/less-mat/';
            outmatpath=lessmatpath;
            imgpath='/home/yanmei/Data/openloris/';

            variation_id=[-1,0,1,2,3];%stastic, illumination, object occlusion/variation, small viewpoint, strong viewpoint

            costerpath=strcat(outmatpath,'Feature_Allcoster_',season,'_',mapseason,'_sift.mat');
            fprintf('loading data...\n');
            Allcoster = load(costerpath);
            Allcoster = Allcoster.Allcoster;

            gtpath=[matpath,season,'gtpose.mat'];
            gtpose=load(gtpath);
            testNum=size(gtpose.Allpose,1);
            mapgtpose=load([matpath,mapseason,'gtpose.mat']);
            fprintf('all on board\n');

            pointTh=8;%308
            % pointTh=400;%308
            % pointTh=625;%308
            iterNum=100;

            easy_ids=[];
            medium_ids=[];
            hard_ids=[];


            % % figure(1);
            % Allerror=[];
            % Allpose=[];
            for costerid=1:length(Allcoster)
            % for costerid=719:719
                if isempty(Allcoster{costerid})
                    continue;
                end
                coster=Allcoster{costerid}(1);
            %     coster=Allcoster{costerid}(maxpos_top20(costerid,1));
            %     if ~ismember(costerid,badids)
            %         Allerror=[Allerror;re.Allerror(costerid,:)];
            %         Allpose=[Allpose;re.Allpose(costerid,:)];
            %         continue;
            %     end
                testid = coster.testid;
                mapid = coster.mapid;
                testname = coster.test_name;
                mapname = coster.map_name;
                pos=strfind(testname,'/');
                test_time_str=testname(pos(end)+1:end-4);
                test_time=str2double(test_time_str);
                map_time_str=mapname(pos(end)+1:end-4);
                map_time=str2double(map_time_str);
                % load ground truth pose
                pos=find(gtpose.Allpose(:,1)==test_time);
                if isempty(pos)
                    continue;
                end
                qpose=gtpose.Allpose(pos,2:8);
                t=qpose(1:3).';
                R=quat2rotm(qpose(4:7));
                T_wb=[R,t; 0 0 0 1];%T_bw
            %     T_cw=Tbc\T_bw;
            %     T_cw=Tcb*T_bw;
                T_wc=T_wb*Tbc;
                T_cw=inv(T_wc);
                R=T_cw(1:3,1:3);
                t=T_cw(1:3,4);
                ay = coster.angles.ay;
                ax = coster.angles.ax;
            %     points3d=coster.points3d(:,inlierIds);
            %     points2d=coster.points2d(:,inlierIds);
                points3d=coster.points3d;
                points2d=coster.points2d;
            %     points3d=[];
            %     points2d=[];
                mpoints2d=coster.mpoints2d;

                pos=find(mapgtpose.Allpose(:,1)==map_time);
                if isempty(pos)
                    continue;
                end
                mpose=mapgtpose.Allpose(pos,2:8);
                mt=mpose(1:3).';
                mR=quat2rotm(mpose(4:7));
                T_wmb=[mR,mt; 0 0 0 1];%T_bw
            %     T_cw=Tbc\T_bw;
            %     T_cw=Tcb*T_bw;
                T_wmc=T_wmb*Tbc;

            %     qq_points2d=[q_points2d(2,:);q_points2d(1,:)];
            %     rr_points2d=[r_points2d(2,:);r_points2d(1,:)];

                fprintf('testid=%d,mapid=%d\n',testid,mapid);



                %%

                model=4;

                if size(points2d,2)<5
                    continue;
                end

                if model==4
                %     % jiao
                %     3p ransac
                    [finalR_,finalt,suc,p3pinliers]=cv.solvePnPRansac(points3d.',points2d(1:2,:).',K,'Method','P3P','IterationsCount',iterNum,...
                        'ReprojectionError',pointTh);
            %         [finalR_,finalt,suc,p3pinliers]=cv.solvePnPRansac(points3d.',points2d(1:2,:).',K,'Method','EPnP','IterationsCount',100);
            %         [finalR_,finalt,finalinliers]=cv.solvePnPRansac(coster.points3d.',coster.points2d.',K);
                    finalR=cv.Rodrigues(finalR_);
                end
                quatError=rotm2axang(R/finalR);%degree
                rotaError=quatError(4);
                rotaError=rotaError/pi*180;%degree
                tranError=norm(t-finalt);
                fprintf('tranError=%f,rotaError=%f\n',tranError,rotaError);
                suc_p3p = tranError<suc_tran_th&rotaError<suc_angle_th;

                model=5;

                 if model==5
                %     % jiao
                %     3p ransac
            %         [finalR_,finalt,suc,p3pinliers]=cv.solvePnPRansac(points3d.',points2d(1:2,:).',K,'Method','P3P','IterationsCount',100);
                    [finalR_,finalt,suc,p3pinliers]=cv.solvePnPRansac(points3d.',points2d(1:2,:).',K,'Method','EPnP','IterationsCount',iterNum,...
                        'ReprojectionError',pointTh);
            %         [finalR_,finalt,finalinliers]=cv.solvePnPRansac(coster.points3d.',coster.points2d.',K);
                    finalR=cv.Rodrigues(finalR_);
                 end
                quatError=rotm2axang(R/finalR);%degree
                rotaError=quatError(4);
                rotaError=rotaError/pi*180;%degree
                tranError=norm(t-finalt);
                fprintf('tranError=%f,rotaError=%f\n',tranError,rotaError);
                suc_epnp = tranError<suc_tran_th&rotaError<suc_angle_th;

           


            %%

                if suc_epnp&&suc_p3p
                    easy_ids=[easy_ids;costerid];
                    disp('easy');
                else
            

                    quatError=rotm2axang(T_wmc(1:3,1:3)/T_wc(1:3,1:3));%degree
                    rotaError=quatError(4)/pi*180;
                    tranError=norm(T_wmc(1:3,4)-T_wc(1:3,4));
                    fprintf('test and map: tranError=%f,rotaError=%f\n',tranError,rotaError);
                    if rotaError<25&&tranError<3
                        medium_ids=[medium_ids;[costerid,rotaError,tranError]];
                        disp('medium');
                    else
                        hard_ids=[hard_ids;[costerid,rotaError,tranError]];
                        disp('hard');
                    end
                end



            %     figure(1);
            %     mapimg = imread(strcat(imgpath,mapname));
            %     testimg = imread(strcat(imgpath,testname));
            %     showMatchedFeatures(testimg,mapimg,points2d(1:2,:).',mpoints2d(1:2,:).','montage');
            %     waitforbuttonpress;



            end

            save([outmatpath,season,'_',mapseason,'_easy_hard_new.mat'],'easy_ids','medium_ids','hard_ids');
        
    end
end
