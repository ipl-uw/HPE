clc;
clear;
close all;

% window size for smoothing the camera positions
cfgSmhWinSz = 31;
% threshold of score of 2D pose estimation
psEstScrThld = 0.10;
% viewing perspective to plot the 3D scene
viewAng = [0,0,-40];
% frame count to plot the 3D scene (-1: process every frame)
frmCntPlt = -1;
% starting frame count to plot the 3D scene
frmCntSt = 0;
% choosing the specific standard
jntPtNum = 18;
% jntPtNum = 15;
jntPtPrCoco = [[1,2];[1,5];[2,3];[3,4];[5,6];[6,7];...
    [1,8];[8,9];[9,10];[1,11];[11,12];[12,13];...
    [1,0];[0,14];[14,16];[0,15];[15,17]];
% jntPtPrMpi = [[0,1];[1,2];[2,3];[3,4];[1,5];[5,6];...
%     [6,7];[1,14];[14,8];[8,9];[9,10];[14,11];...
%     [11,12];[12,13]];
jntPtClrCoco = [[85,0,255];[0,0,255];[0,85,255];[0,170,255];[0,255,255];[0,255,170];...
    [0,255,85];[0,255,0];[85,255,0];[170,255,0];[255,255,0];[255,170,0];...
    [255,85,0];[255,0,0];[170,0,255];[255,0,170];[255,0,255];[0,255,85]];
% jntPtClrMpi = [[85,0,255];[0,0,255];[0,85,255];[0,170,255];[0,255,255];[0,255,170];...
%     [0,255,85];[0,255,43];[0,255,0];[85,255,0];[170,255,0];[255,255,0];...
%     [255,170,0];[255,85,0];[255,0,0]];

% read the camera trajectory from text file
fileIdR = fopen('..\\data\\camParam.txt');
CamPosInfo = textscan(fileIdR,'%d,%f,%f,%f,%f,%f,%f');
fclose(fileIdR);

% smooth the results of camera positions
camTrajLen = size(CamPosInfo{1},1);

if camTrajLen > cfgSmhWinSz
    smhX = zeros(camTrajLen,1); smhX(1) = CamPosInfo{5}(1); smhX(camTrajLen) = CamPosInfo{5}(camTrajLen);
    smhY = zeros(camTrajLen,1); smhY(1) = CamPosInfo{6}(1); smhY(camTrajLen) = CamPosInfo{6}(camTrajLen);
    smhZ = zeros(camTrajLen,1); smhZ(1) = CamPosInfo{7}(1); smhZ(camTrajLen) = CamPosInfo{7}(camTrajLen);
    for idx = 1:(camTrajLen - 2)
        if ((idx >= ((cfgSmhWinSz - 1) / 2)) && (idx < (camTrajLen - ((cfgSmhWinSz - 1) / 2))))
            smhWinSz = cfgSmhWinSz;
        elseif (idx < ((cfgSmhWinSz - 1) / 2))
            smhWinSz = (idx * 2) + 1;
        else
            smhWinSz = ((camTrajLen - idx - 1) * 2) + 1;
        end

        smhWinSumX = 0.0;
        smhWinSumY = 0.0;
        smhWinSumZ = 0.0;

        for smhFrmCnt = (idx - ((smhWinSz - 1) / 2)):(idx + ((smhWinSz - 1) / 2))
            smhWinSumX = smhWinSumX + CamPosInfo{5}(smhFrmCnt+1);
            smhWinSumY = smhWinSumY + CamPosInfo{6}(smhFrmCnt+1);
            smhWinSumZ = smhWinSumZ + CamPosInfo{7}(smhFrmCnt+1);
        end

        smhX(idx+1) = smhWinSumX / smhWinSz;
        smhY(idx+1) = smhWinSumY / smhWinSz;
        smhZ(idx+1) = smhWinSumZ / smhWinSz;
    end
else
    smhX = zeros(camTrajLen,1); 
    smhY = zeros(camTrajLen,1); 
    smhZ = zeros(camTrajLen,1); 
    
    for idx = 0:(camTrajLen - 1)
        smhX(idx+1) = CamPosInfo{5}(idx+1);
        smhY(idx+1) = CamPosInfo{6}(idx+1);
        smhZ(idx+1) = CamPosInfo{7}(idx+1);
    end
end

% read the 3D poses from text file
jntPt3dCoord = zeros(jntPtNum, 4);
jntPt3dCoord_prev = zeros(jntPtNum, 4);
jntPt3dCoord_next = zeros(jntPtNum, 4);
fileIdR = fopen('..\\data\\psEst3d.txt');
HumPos3dInfo = textscan(fileIdR,'%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
% HumPos3dInfo = textscan(fileIdR,'%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(fileIdR);

maxX = max(smhX);
maxY = max(smhY);
maxZ = max(smhZ);

minX = min(smhX);
minY = min(smhY);
minZ = min(smhZ);

for jntPtIdx = 0:(jntPtNum - 1)
    maxJntPtX = max(HumPos3dInfo{jntPtIdx*4+2});
    if (maxJntPtX > maxX)
        maxX = maxJntPtX;
    end
    
    maxJntPtY = max(HumPos3dInfo{jntPtIdx*4+3});
    if (maxJntPtY > maxY)
        maxY = maxJntPtY;
    end
    
    maxJntPtZ = max(HumPos3dInfo{jntPtIdx*4+4});
    if (maxJntPtZ > maxZ)
        maxZ = maxJntPtZ;
    end
    
    minJntPtX = min(HumPos3dInfo{jntPtIdx*4+2});
    if (minJntPtX < minX)
        minX = minJntPtX;
    end
    
    minJntPtY = min(HumPos3dInfo{jntPtIdx*4+3});
    if (minJntPtY < minY)
        minY = minJntPtY;
    end
    
    minJntPtZ = min(HumPos3dInfo{jntPtIdx*4+4});
    if (minJntPtZ < minZ)
        minZ = minJntPtZ;
    end
end

% plot 3D camera locations
mkdir('..\\data\\out3dImg');
fig = figure;
pltIdx = 1;
frmCntNd = CamPosInfo{1}(camTrajLen);

if -1 < frmCntPlt
    frmCntSt = frmCntPlt;
    frmCntNd = frmCntPlt;
end
for frmCnt = frmCntSt:frmCntNd
    if (HumPos3dInfo{1}(pltIdx) == frmCnt)
        disp(frmCnt);
    
        for jntPtIdx = 0:(jntPtNum - 1)
            jntPt3dCoord(jntPtIdx+1, :) = [HumPos3dInfo{jntPtIdx*4+2}(pltIdx),...
                HumPos3dInfo{jntPtIdx*4+3}(pltIdx),...
                HumPos3dInfo{jntPtIdx*4+4}(pltIdx),...
                HumPos3dInfo{jntPtIdx*4+5}(pltIdx)];
            % joint points smoothing
            if (pltIdx > 1 && pltIdx < camTrajLen)
                %jntPt3dCoord(jntPtIdx+1,4) 
                jntPt3dCoord_next(jntPtIdx+1, :)=[HumPos3dInfo{jntPtIdx*4+2}(pltIdx+1),...
                    HumPos3dInfo{jntPtIdx*4+3}(pltIdx+1),...
                    HumPos3dInfo{jntPtIdx*4+4}(pltIdx+1),...
                    HumPos3dInfo{jntPtIdx*4+5}(pltIdx+1)];
            
                jntPt3dCoord_prev(jntPtIdx+1, :)=[HumPos3dInfo{jntPtIdx*4+2}(pltIdx-1),...
                    HumPos3dInfo{jntPtIdx*4+3}(pltIdx-1),...
                    HumPos3dInfo{jntPtIdx*4+4}(pltIdx-1),...
                    HumPos3dInfo{jntPtIdx*4+5}(pltIdx-1)];
                
                if jntPt3dCoord(jntPtIdx+1,4) < psEstScrThld && jntPt3dCoord_next(jntPtIdx+1,4) < psEstScrThld && jntPt3dCoord_prev(jntPtIdx+1,4) < psEstScrThld 
                elseif jntPt3dCoord(jntPtIdx+1,4) < psEstScrThld && jntPt3dCoord_prev(jntPtIdx+1,4) < psEstScrThld
                elseif jntPt3dCoord(jntPtIdx+1,4) < psEstScrThld && jntPt3dCoord_next(jntPtIdx+1,4) < psEstScrThld
                elseif jntPt3dCoord_prev(jntPtIdx+1,4) < psEstScrThld && jntPt3dCoord_next(jntPtIdx+1,4) < psEstScrThld
                
                elseif jntPt3dCoord_prev(jntPtIdx+1,4) < psEstScrThld
                    jntPt3dCoord(jntPtIdx+1, :) = (jntPt3dCoord(jntPtIdx+1, :)+...
                    jntPt3dCoord_next(jntPtIdx+1,:)) /2;                    
                elseif jntPt3dCoord_next(jntPtIdx+1,4) < psEstScrThld
                    jntPt3dCoord(jntPtIdx+1, :) = (jntPt3dCoord(jntPtIdx+1, :)+...
                    jntPt3dCoord_prev(jntPtIdx+1,:)) /2;                     
                elseif jntPt3dCoord(jntPtIdx+1,4) < psEstScrThld
                    jntPt3dCoord(jntPtIdx+1, :) = (jntPt3dCoord_next(jntPtIdx+1, :)+...
                    jntPt3dCoord_prev(jntPtIdx+1,:)) /2;                        
                else
                    jntPt3dCoord(jntPtIdx+1, :) = (jntPt3dCoord(jntPtIdx+1, :)+...
                    jntPt3dCoord_next(jntPtIdx+1,:)+...
                    jntPt3dCoord_prev(jntPtIdx+1,:)) /3;
                end
            end
        end

        clf(fig);
        hold on;
        %plot3(CamPosInfo{5}(1:frmCnt+1),CamPosInfo{6}(1:frmCnt+1),CamPosInfo{7}(1:frmCnt+1));
        plot3(smhX(1:pltIdx),smhY(1:pltIdx),smhZ(1:pltIdx), 'Color', ([0,0,0] / 255), 'LineWidth', 3);
        for jntPtIdx = 0:(jntPtNum - 1)
            if jntPt3dCoord(jntPtIdx+1,4) > psEstScrThld
                scatter3(jntPt3dCoord(jntPtIdx+1,1),jntPt3dCoord(jntPtIdx+1,2),...
                    jntPt3dCoord(jntPtIdx+1,3),...
                    'MarkerFaceColor', (jntPtClrCoco(jntPtIdx+1,1:3) / 255),...
                    'MarkerEdgeColor', (jntPtClrCoco(jntPtIdx+1,1:3) / 255));
            end
            if jntPtIdx > 0
                if jntPt3dCoord((jntPtPrCoco(jntPtIdx,1)+1),4) > psEstScrThld && jntPt3dCoord((jntPtPrCoco(jntPtIdx,2)+1),4) > psEstScrThld
                    plot3([jntPt3dCoord((jntPtPrCoco(jntPtIdx,1)+1),1),jntPt3dCoord((jntPtPrCoco(jntPtIdx,2)+1),1)],...
                        [jntPt3dCoord((jntPtPrCoco(jntPtIdx,1)+1),2),jntPt3dCoord((jntPtPrCoco(jntPtIdx,2)+1),2)],...
                        [jntPt3dCoord((jntPtPrCoco(jntPtIdx,1)+1),3),jntPt3dCoord((jntPtPrCoco(jntPtIdx,2)+1),3)],...
                        'Color', (jntPtClrCoco(jntPtIdx,:) / 255), 'LineWidth', 3);
                    
                end
            end
        end
        hold off;

        axis([minX maxX minY maxY minZ maxZ]);
        %axis equal;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        grid on;
        view(viewAng);
        outImgStr = sprintf('..\\data\\out3dImg\\%06d.jpg',CamPosInfo{1}(pltIdx));
        saveas(fig,outImgStr);
        
        pltIdx = pltIdx + 1;
    end
end