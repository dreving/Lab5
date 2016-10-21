function [centroid] = findLineCandidate(rangeImg)
    pointThreshold = 0;
    sailThreshold = 0;
    errorThreshold =0;
    sailLength = .125;
% remove all points with bad range
    goodOnes = rangeImg > 0.06 & rangeImg < 2.0;
    rangeImg = rangeImg(goodOnes);
    indices = linspace(1,length(goodOnes),length(goodOnes))';
    indices = indices(goodOnes);
% Compute the angles of surviving points
   % th = (indices-1)*(pi/180);
    pointCloud = [indices rangeImg]
    pointCloudXY =RangeImage.irToXy(pointCloud)
    
for i = 1:1:length(pointCloudXY(:,1))
    poi = pointCloudXY(i,:);
    sailIndices = ((pointCloudXY(:,1)-poi(1)).^2 + (pointCloudXY(:,2)-poi(2)).^2) < (sailLength/2)^2
   % sailPoints = pointCloudXY([sailIndices sailIndices])
    x = pointCloudXY(:,1);
    y = pointCloudXY(:,2);
    x= x(sailIndices)
    y = y(sailIndices)
    
if length(x) < pointThreshold
    continue
end
 
%moment of inertia stuff
 Ixx = x' * x;
Iyy = y' * y;
Ixy = - x' * y;
Inertia = [Ixx Ixy;Ixy Iyy] / length(x); 
% normalized
lambda = eig(Inertia);
lambda = sqrt(lambda)*1000.0;
sort(lambda);
%Do checks here 
if lambda(1) > errorThreshold
    continue
end    
if abs(lambda(2) - sailLength) > sailThreshold
    continue
end
break
end
th = atan2(2*Ixy,Iyy-Ixx)/2.0;
centroid = [mean(x) mean(y) th];
end

