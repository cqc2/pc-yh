% [a b c]=size(xx);
% vv = [];v = [];
% for k1 = 1:a
%     for k2 = 1:b
%         for k3 = 1:c
%            v =  xx{k1,k2,k3};
%            if v(1)~=0
%                vv = [vv;v(1:6)'];
%            end
%         end
%     end
% end
hold on;
quiver3(vv(:,1),vv(:,2),vv(:,3),vv(:,4),vv(:,5),vv(:,6));
hold on;
plot3(pointCloudData(:,1),pointCloudData(:,2),pointCloudData(:,3),'r.');
