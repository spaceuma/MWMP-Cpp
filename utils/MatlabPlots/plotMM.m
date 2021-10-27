%% Simple script to visualize the results of the planned motions

addpath(genpath('../../../MP-FB_SLQ'))

clear

% System properties
global d1 a2 a3 d5 d6;
d1 = 0.0895;
a2 = 0.206;
a3 = 0.176;
d5 = 0.0555;
d6 = 0.14;

global width1 width2 width3 width4 width5;
width1 = 0.055;
width2 = 0.02;
width3 = 0.02;
width4 = 0.05;
width5 = 0.02;

global dfx dfy;
dfx = 0.3;
dfy = 0.27;

global zGC xCB yCB zCB;
zGC = 0.202;
xCB = 0.165;
yCB = -0.15;
zCB = 0.028;

global reachabilityDistance;
reachabilityDistance = d1+a2+a3+d5+d6-zGC-zCB+xCB;

global armHeight armWidth;
armHeight = 0.05;
armWidth = 0.1;

global wheelRadius wheelWidth wheelMass;
wheelRadius = 0.07;
wheelWidth = 0.09;
wheelMass = 0.484;

global vehicleMass;
vehicleMass = 15;

global m1 m2 m3 m4 m5;
m1 = 0.5;
m2 = 0.8;
m3 = 0.8;
m4 = 0.3;
m5 = 0.2;

global rollingResistance;
rollingResistance = 0.0036;

global g;
g = 9.81;

% Loading planned state and control
x = importdata("../../test/unit/results/stepped_planned_state.txt").';
u = importdata("../../test/unit/results/stepped_planned_control.txt").';

% Initial state
xei = x(1,1);
yei = x(2,1);
zei = x(3,1);

xC0 = x(10,1);
yC0 = x(11,1);
yawC0 = x(12,1);

% Goal end effector pose
xef = 3.0;
yef = 2.80;
zef = 0.10;
rollef = pi/4;
pitchef = pi/2;
yawef = 0;

% Solution characteristics
timeSteps = 160;
tf = 160; 
dt = tf/(timeSteps-1);
t = 0:dt:tf;

% Distance to obstacles considered risky (should be an obstacle)
riskDistance = 0.30;

% Distance to obstacles considered enough for safety (should have bigger
% cost to traverse areas with less than this distance to obstacles)
safetyDistance = 1.00;

% Reference trajectory computation
% FMM to compute totalCostMap
% Loading obstacles map
% Name of the obst map to be used
obstMapFile = 'obstMap4';

load(obstMapFile,'obstMap')
mapResolution = 0.05;

% Initial spot and goal indexes in the map
iInit = [round(xC0/mapResolution)+1 round(yC0/mapResolution)+1];
iGoal = [round(xef/mapResolution)+1 round(yef/mapResolution)+1];

% Generating a fake obstacle on the sample to avoid stepping on it
obstMap(iGoal(2), iGoal(1)) = 1;

% Dilating obstacles map to ensure rover safety
dilatedObstMap = dilateObstMap(obstMap, riskDistance, mapResolution);
safeObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);

% Generating cost map for FMM
costMap = ones(size(obstMap));
distMap = mapResolution*bwdist(obstMap);
auxMap = 1./distMap;
gradient = 1;
minCost = max(max(costMap(costMap~=Inf)));
costMap(safeObstMap==1)= minCost + gradient*auxMap(safeObstMap==1)./min(min(auxMap(safeObstMap==1)));
costMap(safeObstMap==1)= costMap(safeObstMap==1)./min(min(costMap(safeObstMap==1)));
costMap(obstMap==1) = max(max(costMap(costMap~=Inf)));

% Computing total cost map
[totalCostMap, ~] = computeTmap(costMap,iGoal);

% Computing gradient
totalCostMap(totalCostMap == Inf) = NaN;
[gTCMx, gTCMy] = calculateGradient(mapResolution*totalCostMap);

% Extracting the reference path
[referencePath,~] = getPathGDM(totalCostMap,iInit,iGoal,0.5, gTCMx, gTCMy);

% Ensuring no singularities happened in the gradient
while(size(referencePath,1) > 1000)
    [referencePath,~] = getPathGDM(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
end

% Translating from indexes to meters
referencePath = (referencePath-1)*mapResolution;

% Resizing path to match number of timeSteps
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),timeSteps);
referencePath = interp1(x1,referencePath,x2);

% Arm position constraints
armJointsPositionLimits = [-30 +90;
                   -190 -10;
                   -160 +160;
                   -70 +250;
                   -160 +160]*pi/180; % rad

for i = 2:timeSteps
    if(x(16,i) < armJointsPositionLimits(1,1) || x(16,i) > armJointsPositionLimits(1,2))
        warning(['Arm joint 1 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(17,i) < armJointsPositionLimits(2,1) || x(17,i) > armJointsPositionLimits(2,2))
        warning(['Arm joint 2 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(18,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(19,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end      
    if(x(20,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end      
end

iu = cumsum(abs(x(31,:))*dt);
disp(['Total torque applied arm joint 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(32,:))*dt);
disp(['Total torque applied arm joint 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(33,:))*dt);
disp(['Total torque applied arm joint 3: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(34,:))*dt);
disp(['Total torque applied arm joint 4: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(35,:))*dt);
disp(['Total torque applied arm joint 5: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(40,:))*dt);
disp(['Total torque applied left wheels: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(41,:))*dt);
disp(['Total torque applied right wheels: ',num2str(iu(end)),' Nm'])

%% Visualization stuff
cmap = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(cmap);
xVect = linspace(0,(size(obstMap,1)-1)*mapResolution,size(obstMap,1));
[X,Y] = meshgrid(xVect,xVect);

figure(1)
clf(1)
% Plotting first arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,1));
TWC = getTraslation([x(10,1),x(11,1),zGC])*getZRot(x(12,1));
TCB = getTraslation([xCB, yCB, zCB])*getYRot(pi/2);
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
h1 = plot3([TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
           [TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
           [TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
                         'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
hold on;
    
% Plotting first rover position [TODO] Update the representation to look
% like exoter
TWC = getTraslation([x(10,1),x(11,1),zGC])*getZRot(x(12,1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
h2 = plot3([TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
      [TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
      [TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the first base frame
h3 = plotFrame(TWC, 0.3);

% Plotting last arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,end-1));
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
h4 = plot3([TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
           [TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
           [TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
                         'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
                     
% Plotting last rover position
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
h5 = plot3([TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
          [TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
          [TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the last base frame
h6 = plotFrame(TWC, 0.3);
       
% Plotting starting and goal ee poses
h7 = plot3(xei,yei,zei, 'MarkerSize', 20, 'Marker', '.', 'Color', [0.1 0.1 0.1]);
h8 = plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', [0.1 0.1 0.1]);

% Plotting the reference frame
h9 = plotFrame([1 0 0 0;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1], 1, 'W');
            

% Plotting scenario
h10 = contourf(X,Y,dilatedObstMap+obstMap,0:2);

% Plotting ee path
h11 = plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 1, 'Color', 'y');

% Plotting base path
h12 = plot3(x(10,1:end-1),x(11,1:end-1),zGC*ones(size(x,2)-1), 'LineWidth', 1.5, 'Color', [1,0.5,0]);

% Plotting starting reference path
h14 = plot3(referencePath(:,1),referencePath(:,2), zGC*ones(timeSteps,2), 'LineWidth', 1, 'Color', [0,0,1]);

title('Mobile manipulator trajectories', 'interpreter', ...
'latex','fontsize',18)
daspect([1 1 1])
     
hold off;
figure(1);        
% Plotting last arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,end-1));
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
set(h4,'XData',[TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
       'YData',[TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
       'ZData',[TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
       'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);

% Plotting last rover position
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
set(h5,'XData',[TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
       'YData',[TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
       'ZData',[TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)],...
       'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the last base frame
h6 = plotFrame(TWC, 0.3, 0, h6);


% Plotting ee path
set(h11,'XData',x(1,:),'YData',x(2,:),'ZData',x(3,:),...
    'LineWidth', 1, 'Color', 'y');

% Plotting base path
set(h12,'XData',x(10,1:end-1),'YData',x(11,1:end-1),'ZData',zGC*ones(size(x,2)-1,1),...
    'LineWidth', 1.5, 'Color', [1,0.5,0]);

hold off; 
        
figure(2)
plot(t,x(16:20,:))
title('Evolution of the arm joints position', 'interpreter', ...
'latex','fontsize',18)
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid 
