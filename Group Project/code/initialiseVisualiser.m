function [armPartPatchHandle, armPartVertices, pendPartPatchHandle, pendPartVertices, drivePartPatchHandle, drivePartVertices, thetaPlotHandle, alphaPlotHandle] = initialiseVisualiser(lengthOfReplay)
FIG = figure(1);
clf;
set(gcf,'color','w');

%
% Set up a pictorial plot on the left.
%
sp1 = subplot(1,3,1);
clc;
hold on;
grid on;
axis equal;

fs = 20;
set(gca,'fontsize',fs);
xlabel('X [m]','interpreter','latex','fontsize',fs);
ylabel('Y [m]','interpreter','latex','fontsize',fs);
zlabel('Z [m]','interpreter','latex','fontsize',fs);

edgecolor = 'none';
facealpha = 1;
axis([-0.22, 0.22, -0.22, 0.22, 0, 0.45]);

thetaAngle = 0;
alphaAngle = 0;

%
% Frame transformations.
%
BaseToArm = homogeneous_func(0,0,thetaAngle*pi/180,0,0,12e-2);
ArmToPendulum = homogeneous_func(alphaAngle*pi/180,0,0,14.2e-2,0,5e-2);
BaseToDrive = homogeneous_func(0,0,(-thetaAngle*5)*pi/180,0,0.036,12e-2);

%
% Base parts.
%
clear VRML_vertices;
load baseParts_vertices;
[X Y Z C] = vrml2patch_func(size(VRML_vertices,2),VRML_vertices);
X = X*10; Y = Y*10; Z = Z*10;
basePartVertices = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';
basePart_vertices_G_frame = eye(4) * basePartVertices;
Xt = reshape(basePart_vertices_G_frame(1,:),3,[]);
Yt = reshape(basePart_vertices_G_frame(2,:),3,[]);
Zt = reshape(basePart_vertices_G_frame(3,:),3,[]);
basePartPatchHandle = patch(Xt,Yt,Zt,C,'Edgecolor', edgecolor,'facealpha',facealpha,'clipping','off');

%
% Arm parts.
%
clear VRML_vertices;
load armParts_vertices;
[X Y Z C] = vrml2patch_func(size(VRML_vertices,2),VRML_vertices);
X = X*10; Y = Y*10; Z = Z*10;
armPartVertices = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';
armPart_vertices_G_frame = BaseToArm * armPartVertices;
Xt = reshape(armPart_vertices_G_frame(1,:),3,[]);
Yt = reshape(armPart_vertices_G_frame(2,:),3,[]);
Zt = reshape(armPart_vertices_G_frame(3,:),3,[]);
armPartPatchHandle = patch(Xt,Yt,Zt,C,'Edgecolor', edgecolor,'facealpha',facealpha,'clipping','off');

%
% Pend parts.
%
clear VRML_vertices;
load pendParts_vertices;
[X Y Z C] = vrml2patch_func(size(VRML_vertices,2),VRML_vertices);
X = X*10; Y = Y*10; Z = Z*10;
pendPartVertices = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';
pendPart_vertices_G_frame = BaseToArm * ArmToPendulum* pendPartVertices;
Xt = reshape(pendPart_vertices_G_frame(1,:),3,[]);
Yt = reshape(pendPart_vertices_G_frame(2,:),3,[]);
Zt = reshape(pendPart_vertices_G_frame(3,:),3,[]);
pendPartPatchHandle = patch(Xt,Yt,Zt,C,'Edgecolor', edgecolor,'facealpha',facealpha,'clipping','off');

%
% Drive parts.
%
clear VRML_vertices;
load driveParts_vertices;
[X Y Z C] = vrml2patch_func(size(VRML_vertices,2),VRML_vertices);
X = X*10; Y = Y*10; Z = Z*10;
drivePartVertices = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';
drivePart_vertices_G_frame = BaseToDrive* drivePartVertices;
Xt = reshape(drivePart_vertices_G_frame(1,:),3,[]);
Yt = reshape(drivePart_vertices_G_frame(2,:),3,[]);
Zt = reshape(drivePart_vertices_G_frame(3,:),3,[]);
drivePartPatchHandle = patch(Xt,Yt,Zt,C,'Edgecolor', edgecolor,'facealpha',facealpha,'clipping','off');

view(144,26);
material dull;
lightX = 3;
lightY = 1;
lightZ = 2;
light('Position',[lightX, lightY,lightZ]);
light('Position',-[lightX, lightY,lightZ]);


%
% Set up the raw data plots on the right.
%
sp2 = subplot(1,3,2);
clc;
hold on;
grid on;
box on;

set(gca,'fontsize',fs);
thetaPlotHandle = plot(inf,inf,'-r','linewidth',2);
thetaPlotHandleCurrent = plot(inf,inf,'.r','markersize',30);

xlabel('Time [sec]','interpreter','latex','fontsize',fs);
ylabel('$\theta$ [$^\circ$]','interpreter','latex','fontsize',fs);

axis([0, lengthOfReplay, -40, 40]);

sp3 = subplot(1,3,3);
clc;
hold on;
grid on;
box on;

set(gca,'fontsize',fs);
alphaPlotHandle = plot(inf,inf,'-b','linewidth',2);
alphaPlotHandleCurrent = plot(inf,inf,'.b','markersize',30);

xlabel('Time [sec]','interpreter','latex','fontsize',fs);
ylabel('$\alpha$ [$^\circ$]','interpreter','latex','fontsize',fs);

axis([0, lengthOfReplay, -40, 40]);

%
% Position the three subplots onto the canvas.
%
set(sp1,'position', [0.07 0.14 0.40 0.83]);
set(sp2,'position', [0.55 0.60 0.43 0.33]);
set(sp3,'position', [0.55 0.10 0.43 0.33]);


%
% Package up plot handles for updating.
%
% visualiserPlotHandles.armPartPatchHandle = armPartPatchHandle;
% visualiserPlotHandles.armPartVertices = armPartVertices;
% visualiserPlotHandles.pendPartPatchHandle = pendPartPatchHandle;
% visualiserPlotHandles.pendPartVertices = pendPartVertices;
% visualiserPlotHandles.drivePartPatchHandle = drivePartPatchHandle;
% visualiserPlotHandles.drivePartVertices = drivePartVertices;
% visualiserPlotHandles.thetaPlotHandle = thetaPlotHandle;
% visualiserPlotHandles.alphaPlotHandle = alphaPlotHandle;

end



function [T] = homogeneous_func(ROLL, PITCH, YAW, X, Y, Z)
T=zeros(4,4);
T(1,1) = cos(YAW)*cos(PITCH);
T(1,2) = cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL);
T(1,3) = cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL);
T(1,4) = X;
T(2,1) = sin(YAW)*cos(PITCH);
T(2,2) = sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL);
T(2,3) = sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL);
T(2,4) = Y;
T(3,1) = -sin(PITCH);
T(3,2) = cos(PITCH)*sin(ROLL);
T(3,3) = cos(PITCH)*cos(ROLL);
T(3,4) = Z;
T(4,4) = 1;
end


function [X Y Z C] = vrml2patch_func(nel, w3d)
% VRML2PATCH - VRML struct conversion
% Usage [X Y Z C] = vrml2patch(nel, w3d)
% creates X Y Z and C matrices for patch function from the w3d struct (and
% number of elements nel) given by the read_vrml function.
%
% See also READ_VRML, PATCH
[X Y Z c] = deal([]);
for i = 1:nel
    vertex = [];
    num = w3d(i).polynum;
    points = w3d(i).pts(w3d(i).knx(:), :);
    vertex(:,:, 1) = points(1:num,:);
    vertex(:,:, 2) = points(num+1:2*num,:);
    vertex(:,:, 3) = points(num*2+1:end,:);
    
    X = [X squeeze(vertex(:, 1, :))'];
    Y = [Y squeeze(vertex(:, 2, :))'];
    Z = [Z squeeze(vertex(:, 3, :))'];
    c = [c; repmat(w3d(i).color,num,1)];
end
C(1,:,:) = c;
end
