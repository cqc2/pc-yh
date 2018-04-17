function [voxelArray,outMesh]= voxelpoint(pointCloudData,gridSize)
%generate voxel of point cloud

    [width,height,minX,minY,maxX,maxY,minZ,maxZ,altitude] = calculatesize(pointCloudData,gridSize);
    widthStripsArray = cut2strips(pointCloudData,width,minX,maxX,gridSize,1);
    voxelArray = cell(height,width,altitude);
    outMesh = zeros(size(voxelArray));
    for i = 1:width
        widthStrip = widthStripsArray{i};
       heightStripsArray= cut2strips(widthStrip,height,minY,maxY,gridSize,2);
       for m = 1:height
           altitudeStrip = heightStripsArray{m};
           [altitudeStripsArray ,meshGrid]= cut2strips(altitudeStrip,altitude,minZ,maxZ,gridSize,3);
           voxelArray(i,m,:) = altitudeStripsArray';
           outMesh(i,m,:) = meshGrid';
       end
    end
end

function [stripsArray,meshGrid] = cut2strips(pointData,nStrips,startValue,endValue,pxielSize,type)
%cut point into strips
%type==1, cut by x coordinate;
%type==2, cut by y coordinate;
%type==3, cut by z coordinate;
    stripsArray(1:nStrips) = {[]};
    meshGrid = zeros(1,nStrips);
    if isempty(pointData)
        return;
    end
    pointData = sortrows(pointData,type);%按x坐标排序
    nPoint = size(pointData,1);
    valueArray = pointData(:,type);%分割的依据，如按x或者y坐标
    cutStart = startValue;
    cutEnd = startValue + pxielSize;
    iPoint=1;
    value = valueArray(1);
    isEndPoint = false;%是否遍历到最后一个点
    for i = 1:nStrips %分成nStrips条
        strip = [];
        iStripPoint = 0;
        while value<cutEnd
            iStripPoint = iStripPoint+1;
            strip(iStripPoint,:) = pointData(iPoint,:);
            if iPoint<nPoint
                iPoint = iPoint+1;   
                value = valueArray(iPoint);
            else
                isEndPoint = true;
                break;
            end
        end  
        stripsArray(i) = {strip};
        meshGrid(i) = size(strip,1);
        cutStart = cutEnd;
        cutEnd = cutEnd + pxielSize;
        if isEndPoint
            break;
        end
    end
end

function [width,height,minX,minY,maxX,maxY,minZ,maxZ,altitude] = calculatesize(pointCloudData,pxielSize)
%calcullate width and height of inage
xAraay = pointCloudData(:,1);
yArray = pointCloudData(:,2);
minX = min(xAraay);
maxX = max(xAraay);
minY = min(yArray);
maxY = max(yArray);
width =  ceil((maxX - minX)/pxielSize);
height = ceil((maxY - minY)/pxielSize);

if size(pointCloudData,2)>=3
    zArray = pointCloudData(:,3);
    minZ = min(zArray);
    maxZ = max(zArray);
    altitude = ceil((maxZ - minZ)/pxielSize);
else
    minZ = [];
    maxZ = [];
    altitude = [];
end
end