clc
clear

% starting and ending frames
frmCntSt=5;
frmCntNd=605;

mkdir('..\\data\\demo');

for frmCnt = frmCntSt:frmCntNd
    disp(frmCnt);
    
    % read multiple input images
    inImgStr0 = sprintf('..\\data\\outImg\\%06d.jpg',frmCnt);
    inImg0 = imread(inImgStr0);
    row_Img0 = size(inImg0,1)
    column_Img0 = size(inImg0,2)
    % read 3d plot images
    inImgStr1 = sprintf('..\\data\\out3dImg\\%06d.jpg',frmCnt);
    inImg1 = imread(inImgStr1);
    row_Img1 = size(inImg1,1);
    column_Img1 = size(inImg1,2);
    % resize 3d plot images to merge with original images 
    inImg1 = imresize(inImg1, [row_Img0, row_Img0/row_Img1*column_Img1]);
    % inImg1 = imresize(inImg1, (row_Img0/row_Img1));  % Original Resize
    column_Img1 = size(inImg1,2)
    % merge with original images 
    outImg = zeros(row_Img0,column_Img0+column_Img1,3);
    outImg(1:row_Img0,1:column_Img0,1:3) = inImg0;
    outImg(1:row_Img0,(column_Img0):(column_Img0+column_Img1-1),1:3) = inImg1;

    % write output images
    outImgStr = sprintf('..\\data\\demo\\%06d.jpg',frmCnt);
    imwrite(uint8(outImg), outImgStr);
end

disp('finished');