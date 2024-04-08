function[] = skeletonization_test(bimg) %#codegen

%% Read binary image of robot
%bimg = imread("~/Downloads/images/image_capture_31.png");
img = imcomplement(bimg);
imgbw = imbinarize(img);

%% Skeletonize binary image
skel = bwskel(imgbw);
%skel2 = bwskel(imgbw,'MinBranchLength',200); %pruning

%% Overlay skeleton on binary image and display
%figure()
%imshow(labeloverlay(img,skel,'Transparency',0))

%% Skeleton and robot image side by side
% figure()
% montage({img, skel}, 'BackgroundColor', 'blue', 'BorderSize', 5)

end