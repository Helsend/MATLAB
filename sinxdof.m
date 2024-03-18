%%  plot
clc
clear;
close all

%定义连杆的 D-H 参数
L1=Link('d',0.144,'a',0,'alpha',0,'modified');
L2=Link('d',0,'a',0,'alpha',pi/2,'offset',-pi/2,'modified');
L3=Link('d',0,'a',-0.264,'alpha',0,'modified');
L4=Link('d',0.106,'a',-0.236,'alpha',0,'offset',-pi/2,'modified');
L5=Link('d',0.114,'a',0,'alpha',pi/2,'modified');
L6=Link('d',0.067,'a',0,'alpha',-pi/2,'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6');
% L(1)=Link('revolute','d',0.216,'a',0,'alpha',pi/2);
% L(2)=Link('revolute','d',0,'a',0.5,'alpha',0,'offset',pi/2);
% L(3)=Link('revolute','d',0,'a',sqrt(0.145^2+0.42746^2),'alpha',0,'offset',-atan(427.46/145));
% L(4)=Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',atan(427.46/145));
% L(5)=Link('revolute','d',0.258,'a',0,'alpha',0);
% Five_dof=SerialLink(L,'name','5-dof');
% Five_dof.base=transl(0,0,0.28);
% subplot(1,2,1)
% plot(Five_dof,[0 0 0 0 0],'tilesize',0.1,'workspace',[-1 1 -1 1 -0.2 2])
% robot.teach

%% 
q0=[0 0 0 0 0 0];
v=[35 20];
w=[-1 1 -1 1 0 2];
vertice=[-0.8 -1 0.23;-0.8 0 0.23;-0.4 0 0.23;-0.4 -1 0.23;...
         -0.8 -1 0.2;-0.8 0 0.2;-0.4 0 0.2;-0.4 -1 0.2];
face=[1 2 3 4;1 2 6 5;1 4 8 5;2 3 7 6;3 4 8 7];
% set(gcf,'position',[500,150,800,500])
% subplot(1,2,2)
view(3)
robot.plot(q0)
L1=light('Position',[1 1 1],'color','w');

%%   pick and put place

% T1=transl(0.6,0,0.038)*rpy2tr([0 90 -90]);
T1=transl(0.6,0,0.038)*trotx(pi);
q1=robot.ikunc(T1);
qt1=jtraj(q0,q1,50);
% set(gcf,'position',[500,150,800,500])
plot_sphere([0.6,0,0.038],0.08,'g');
patch('Vertices',vertice,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
view(3)
robot.plot(qt1);

T2=transl(-1,-1,0.5)*trotx(pi);
q2=robot.ikunc(T2);
qt2=jtraj(q1,q2,50);

for i=1:size(qt2,1)
    clf
    T=robot.fkine(qt2(i,:));
    P=transl(T);
    plot_sphere(P,0.08,'g')
    patch('Vertices',vertice,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
    view(3)
    robot.plot(qt2(i,:));
    pause(0)
end

clf
qt3=jtraj(q2,q0,50);
plot_sphere(P,0.08,'g')
patch('Vertices',vertice,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
view(3)
robot.plot(qt3);

% %%  plot trail
% figure
% set(gcf,'position',[500,150,800,500])
% axis(w)
% 
q=[qt1;qt2;qt3;q0];
x=[];
y=[];
z=[];
for i=1:1:length(q)
    [x1,y1,z1]=transl(robot.fkine(q(i,:)));
    x=[x;x1];
    y=[y;y1];
    z=[z;z1];
end
hold on
plot3(x,y,z,'b--o','LineWidth',1,'MarkerSize',6,'MarkerEdgeColor','k','MarkerFaceColor','g')
patch('Vertices',vertice,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
% % Five_dof.plot3d([qt1;qt2;qt3],'view',v,'nowrist','delay',0,'fps',30,'trail',{'r','LineWidth',3},'tilesize',0.1)
% hold on
% Five_dof.plot([qt1;qt2;qt3])
