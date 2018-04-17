function demo(pcdFilePathOrData,minH,maxH,interval)
%
%
% 
% The program is written by Chen Qichao. You can redistribute or modify the
% program for any properties.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Chen Qichao
% 2018.04.16

if size(minH,2)>1
    for i=1:2:size(minH,2)
        process(pcdFilePathOrData,minH(i),minH(i+1));
    end
else
    process(pcdFilePathOrData,minH,maxH,interval);
end
end

function process(pcdFilePathOrData,minH,maxH,interval)
%
%参数设置
gridSize = 0.1;%格网大小,建议0.05或者0.1
denseLimit = 3000;%投影密度
sigma  = 5;%滤波参数
para = 0.5;
lLimit = 1;%长度阈值，滤除较小的点云聚落
cdist = 0.3;%聚类距离

% 判断输入的是文件路径还是点云矩阵
[row,col] = size(pcdFilePathOrData);
if row>1&&col>=4
    pcd = pcdFilePathOrData;
elseif row==1
    [path,filename,filetype]=fileparts(pcdFilePathOrData);
    if(filetype=='.las')
        A = LASreadAll(pcdFilePathOrData);
        pcd=[A.x,A.y,A.z,A.intensity];
        savepointcloud2file(pcd,filename,false);
    elseif(filetype=='.xyz')
        fid=fopen(pcdFilePathOrData,'r');
        pcd = readpointcloudfile2(pcdFilePathOrData);%读取全部点
%         pointCloudData =  readpointcloudfile(fid,10000000);%读取指定个数点
    else
        return;
    end
else 
    return;
end

if ~exist('minH','var')||isempty(minH)  minH = min(pcd(:,3));end
if ~exist('maxH','var')||isempty(maxH)  maxH = max(pcd(:,3));end
if ~exist('interval','var')||isempty(interval)  interval = maxH-minH;end
denseLimit = denseLimit*(interval/0.5);

% gridesize = 0.1;
% denseLimit = 3500;
numLimit = 3500*gridSize*gridSize;
for H1 = minH:interval:(maxH-interval)
    H2 = H1+interval;
    data = pcd(pcd(:,3)>=H1&pcd(:,3)<H2,:);
    [gridArray,outMmesh] = gridpoint(data,gridSize);
    outMmesh0 = outMmesh;
    
    filterMesh = imgaussfilt(outMmesh,sigma);%高斯滤波，减小地面密度不均匀影响
    outMmesh = outMmesh-filterMesh.*para;
    
%     mesh(outMmesh);
outMmesh(outMmesh<numLimit) = 0;
outMmesh(outMmesh>0)= 1;

[row,col] = find(outMmesh>0);
index = clustereuclid([row col],ceil(cdist/gridSize));
nc = unique(index);
cluster = struct([]);
for i=1:size(nc,1)
    tmpx = col(index==i);
    tmpy = row(index==i);
    cluster(i).data = [tmpx,tmpy];
%     figure(11);plot(tmpx,tmpy,'.','Color',[rand rand rand]);hold on;axis equal;
end
if size(nc,1)==0
    continue;
end
cluster = getclusterinfo(cluster);

 outMmesh2 = zeros(size(outMmesh));
 p = [];%像素点坐标
for i=1:size(cluster,2)
    length = cluster(i).length;
    if length>(lLimit/gridSize)
        p = [p;cluster(i).data];
%       figure(11);plot(cluster(i).data(:,1),cluster(i).data(:,2),'.','Color',[rand rand rand]);hold on;axis equal;
    end
end
if size(p,1)==0
    continue;
end
zeroIdx = (p(:,1)-1).*size(outMmesh,1)+p(:,2);
 outMmesh2(zeroIdx) = 1;
 
 point = getpointfromgrid(gridArray,outMmesh,1,12);
 point2 = getpointfromgrid(gridArray,outMmesh2,1,12);
 
 strh1 = num2str(H1);
 strh2 = num2str(H2);
    savepointcloud2file(data,strcat(strh1,'~',strh2,'-仅切片'),0);%切片点云
    savepointcloud2file(point,strcat(strh1,'~',strh2,'-密度滤波'),0);%密度滤波点云
    savepointcloud2file(point2,strcat(strh1,'~',strh2,'-长度滤波'),0);%长度滤波点云
    
%     %法向量滤波
%     [voxelArray,~]= voxelpoint(point2,gridSize);
%     nv = cellfun(@normnd,voxelArray,'UniformOutput', false);
    
%     figure(2);mesh(outMmesh0);axis equal;
%     figure(3);plot(data(:,1),data(:,2),'r.');axis equal;
%     figure(4);plot(point(:,1),point(:,2),'r.');axis equal;
%     figure(5);plot(point2(:,1),point2(:,2),'r.');axis equal;
%     
%     a=0;
end
end

function point = getpointfromgrid(gridArray,seg_I,num,type)
%
% -seg_I:整数化后的图像矩阵
% -num;要提取的数字区域
% -type:横纵坐标所在位数，12代表在1、2位，56代表在5、6位。
[row,col] = size(seg_I);
point = zeros(10000,4);
np = 0;
for m = 1:row
    for n = 1:col
        if seg_I(m,n)~=num
            continue;
        end
        p = gridArray{m,n};
        if (type==56)&&(~isempty(p))
            x = p(:,5);
            y = p(:,6);
            h = p(:,3);
            ins = p(:,4);
            p = [x y h ins]; 
        elseif type==12&&(~isempty(p))
            x = p(:,1);
            y = p(:,2);
            h = p(:,3);
            ins = p(:,4);
            p = [x y h ins];
        end       
        if seg_I(m,n)==num&&(~isempty(p))
            preNp = np;
            np = np + size(p,1);
            point(preNp+1:np,:) = p;
        end        
    end
end
point = point(1:np,:);
end
