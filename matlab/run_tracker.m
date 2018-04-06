function [stateInfo, speed] = run_tracker(curSequence, baselinedetections)

%% setting parameters for tracking not used for cpp version
%{ 
c_en      = 10;     %% 10 birth cost, has no influence on the result
c_ex      = 10;     %% 10 death cost, has no influence on the result
c_ij      = -2;     %% 0 transition cost
betta     = 0.01;   %% 0.2 betta, increase will have less tracks, for every single detection
max_it    = inf;    %% inf max number of iterations (max number of tracks)
thr_cost  = 18;     %% 18 max acceptable cost for a track (increase it to have more tracks.), for every tracklet 19.8
%}
%% Necessary parameters
imgPath = curSequence.imgFolder;
imgPath = imgPath(1:end-1);
frameNums = curSequence.frameNums;
seqID = curSequence.seqName;

%% Create detection file to use as input for GOG
detPath = 'detections.txt'; % detection file          
% Write the detections in file for the tracker to read it    
dlmwrite(detPath, baselinedetections);

%% Running tracking algorithm
time_start = tic;
frameNums = curSequence.frameNums;
command = ['cppGOG.exe ' seqID ' ' imgPath];
[status, output] = system(command);
totalTime = toc(time_start);
speed = numel(frameNums)/totalTime;          

%% Save tracking results
stateInfo = [];                
stateInfo.F = numel(curSequence.frameNums);
stateInfo.frameNums = curSequence.frameNums;
if(exist(['result\' seqID '_LX.txt'],'file'))
    X = load(['result\' seqID '_LX.txt']);
    Y = load(['result\' seqID '_LY.txt']);
    W = load(['result\' seqID '_W.txt']);
    H = load(['result\' seqID '_H.txt']);
%    totalTime = load(['result\' seqID '_speed.txt']);      
    xc = X + W/2;
    yc = Y + H/2;
    % foot position
    stateInfo.X = xc;
    stateInfo.Xi = xc;
    stateInfo.Y = yc + H/2;
    stateInfo.Yi = yc + H/2;
    stateInfo.H = H;
    stateInfo.W = W;
    speed = numel(frameNums)/totalTime;
else
    stateInfo.X = [];       
    stateInfo.Y = [];
    stateInfo.H = [];
    stateInfo.W = [];             
    stateInfo.Xi = stateInfo.X;
    stateInfo.Yi = stateInfo.Y;                       
    speed = 0;
end
%% Delete temp files created for execution
delete('result\*.txt');
delete('detections.txt'); 