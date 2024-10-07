% -------------------------------------------------------------------------
% File : DWA 算法
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
% Author :Yuncheng Jiang
% License : Modified BSD Software License Agreement
% 出处：https://b23.tv/rGKUTW - b站up主�?�WHEELTEC�?
% 源代码已经配备丰富的注释，我在其基础上添加了�?些个人理解�??
%               —�?? 2021/10/30  Poaoz 
% -------------------------------------------------------------------------

% 流程梳理 - dwa动�?�窗口算�?
%   1）设置初始化参数：起点�?�终点�?�障碍物、小车的速度加�?�度限制�?
%   2）根据小车当前状态及参数，计算出小车接下来一小段时间可达到的状�?�（主要为�?�度、加速度范围�?
%   3）根据上述计算�?�得的�?�度、加速度，模拟出小车接下来一小段时间可达到的路径
%   4) 借助评价函数，对上述路径进行评估，并选取出最优解，然后使小车执行（执行对应的速度、角速度�?
%   5）再以小车新的位置及状�?�为基础，重复上述�??2-5”，直到判断出小车到达终点�??

%  闲谈：前面学习了RRT、A*、人工势能法，综合来看，这几种方法的套路是类似的�?
%  相比较，DWA更加灵活，无�?栅格化地图并且更贴合小车运动实际�?


% 该函数相当于dwa算法的main函数，内容包�? 参数设定、流程的梳理、绘�? �?
function [] = dwa_V_1_0()
close all;
clear ;
disp('Dynamic Window Approach sample program start!!')
%% 机器人的初期状�?�[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% x=[0 0 pi/2 0 0]'; % 5x1矩阵 列矩�?  位置 0�?0 航向 pi/2 ,速度、角速度均为0
x = [0 0 pi/10 0 0]'; 
% 下标宏定�? 状�?�[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人�?�度
W_ANGLE_SPD = 5;  %机器人角速度 
goal = [7,8];   % 目标点位�? [x(m),y(m)]
% 障碍物位置列�? [x(m) y(m)]
obstacle=[%0 2;
          3 10*rand(1);
%           4 4;
%          5 4;
%            5 5;
          6 10*rand(1);
%          5 9
%          7 8
          8 10*rand(1);
          2 5;      
          4 2;
          7 7;
          9 9
            ];
%边界障碍物，防止跑出图外
 for i =-1
    for j = -1:12
        obstacle = [obstacle; [i,j]];
    end
 end     
for i =12
    for j = -1:12
        obstacle = [obstacle; [i,j]];
    end
end 
for j =-2
    for i = -1:12
        obstacle = [obstacle; [i,j]];
    end
end 
for j=13
    for i= -1:12
        obstacle = [obstacle; [i,j]];
    end
end 
 
obstacleR = 0.5;% 冲突判定用的障碍物半�?
global dt; 
dt = 0.1;% 时间[s]   每一条计算得到的路径，由多个点组�?  dt即为每个点之间的时间间隔
% evalParam[4]/dt+1 = 每条路径的构成点数目   这两个参数更改后，dwa算法的具体效果也将有�?变化

% 机器人运动学模型参数
% �?高�?�度m/s],�?高旋转�?�度[rad/s],加�?�度[m/ss],旋转加�?�度[rad/ss],
% 速度分辨率[m/s],转�?�分辨率[rad/s]]
Kinematic = [1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];    % 调用函数里面�? model
%定义Kinematic的下标含�?              % Kinematic 在路径计算相关函数中，大量用�?
MD_MAX_V    = 1;%   �?高�?�度m/s]
MD_MAX_W    = 2;%   �?高旋转�?�度[rad/s]
MD_ACC      = 3;%   加�?�度[m/ss]
MD_VW       = 4;%   旋转加�?�度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转�?�分辨率[rad/s]]

% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重�?�距离得分的比重、�?�度得分的比重�?�向前模拟轨迹的时间
evalParam = [0.045, 0.1 ,0.1, 3.0];
% evalParam = [2, 0.2 ,0.2, 3.0];
area      = [-3 14 -3 14];% 模拟区域范围 [xmin xmax ymin ymax]

% 模拟实验的结�?
result.x=[];   %累积存储走过的轨迹点的状态�??
tic; % 估算程序运行时间�?�?
flag_obstacle = [1-2*rand(1) 1-2*rand(1) 1-2*rand(1)];
vel_obstacle = 0.05;
temp = 0;
abc = 0;
%movcount=0;

%% Main loop   循环运行 5000�? 指导达到目的�? 或�?? 5000次运行结�?
for i = 1:5000  
    % DWA参数输入 返回控制�? u = [v(m/s),w(rad/s)] �? 轨迹  �? 即机器人将采用的控制参数
    [u,traj] = DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR); % 算出下发速度u/当前速度u
    x = f(x,u); % 机器人移动到下一个时刻的状�?�量 根据当前速度和角速度推导 下一刻的位置和角�?
    abc = abc+1;
    % 历史轨迹的保�?
    result.x = [result.x; x'];  %�?新结�? 以行的形�? 添加到result.x，保存的是所有状态参数�?�，包括坐标xy、朝向�?�线速度、角速度，其实应该是只取坐标就OK
    
    % 是否到达目的�?
    if norm(x(POSE_X:POSE_Y)-goal')<0.25   % norm函数来求得坐标上的两个点之间的距�?
        disp('==========Arrive Goal!!==========');break;
    end
    
    %====Animation====
    hold off;               % 关闭图形保持功能�? 新图出现时，取消原图的显示�??
    ArrowLength = 0.5;      % 箭头长度
    
    % 机器�? �? 绘图操作 
    % quiver(x,y,u,v) �? x �? y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
    quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)),'ok'); 
    % 绘制机器人当前位置的航向箭头
    hold on;                                                     
    %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基�?上，并自动调整坐标轴的范�?
    
    plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位�? �?有历史数据的 X、Y坐标
    plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
    for j = 1:3
        if obstacle(j,2) > 10 && flag_obstacle(j) > 0 || obstacle(j,2) < 0 && flag_obstacle(j) < 0
            flag_obstacle(j) = -flag_obstacle(j);
        end
%        obstacle(j,2)=obstacle(j,2)+flag_obstacle(j)*vel_obstacle;
    end
    
    %plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制�?有障碍物位置
    DrawObstacle_plot(obstacle,obstacleR);
    
    % 探索轨迹 画出待评价的轨迹
    if ~isempty(traj) %轨迹非空
        for it=1:length(traj(:,1))/5    %计算�?有轨迹数  traj �?5行数�? 表示�?条轨迹点
            ind = 1+(it-1)*5; %�? it 条轨迹对应在traj中的下标 
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  %根据�?条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的�?有x坐标�?  traj(ind+1,:)表示第ind条轨迹的�?有y坐标�?
        end
    end
    
    axis(area); %根据area设置当前图形的坐标范围，分别为x轴的�?小�?�最大�?�，y轴的�?小最大�??
    grid on;
    drawnow limitrate;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，�?要使用这个语句�??
    for j = 1:3
        if norm(obstacle(j,:)-x(1:2)')-obstacleR < 0
           disp('==========Hit an obstacle!!==========');
           temp = 1;
           break;
        end
    end
    if temp == 1
        break;
    end
   % movcount = movcount+1;
   % mov(movcount) = getframe(gcf);%  记录动画�?
end
toc;  %输出程序运行时间  形式：时间已�? ** 秒�??
disp(abc)
%movie2avi(mov,'movie.avi');  %录制过程动画 保存�? movie.avi 文件

%% 绘制�?有障碍物位置   ok
% 输入参数：obstacle �?有障碍物的坐�?   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
 x = r * cos(theta) + obstacle(id,1); 
 y = r  *sin(theta) + obstacle(id,2);
 plot(x,y,'-m'); 
end
 %plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % 绘制�?有障碍物位置
 
%% DWA算法实现     ok
% model  机器人运动学模型  �?高�?�度[m/s],�?高旋转�?�度[rad/s],加�?�度[m/ss],旋转加�?�度[rad/ss], 速度分辨率[m/s],转�?�分辨率[rad/s]]
% 输入参数：当前状态�?�模型参数�?�目标点、评价函数的参数、障碍物位置、障碍物半径
% 返回参数：控制量 u = [v(m/s),w(rad/s)] �? 轨迹集合 N * 31  （N：可用的轨迹数）
% 选取�?优参数的物理意义：在�?部导航过程中，使得机器人避开障碍物，朝着目标以较快的速度行驶�?
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% Dynamic Window [vmin,vmax,wmin,wmax] �?小�?�度 �?大�?�度 �?小角速度 �?大角速度速度
Vr = CalcDynamicWindow(x,model);  % 1)根据当前状�?? �? 运动模型 计算当前的参数允许范�?
% 评价函数的计�? evalDB N*5  每行�?组可用参�? 分别�? 速度、角速度、航向得分�?�距离得分�?��?�度得分
%               trajDB      �?5行一条轨�? 每条轨迹都有状�?�x点串组成
[evalDB,trajDB]= Evaluation(x,Vr,goal,ob,R,model,evalParam);  % 2)evalParam 评价函数参数 [heading,dist,velocity,predictDT]
if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end
% 各评价函数正则化
evalDB = NormalizeEval(evalDB);
% 3)�?终评价函数的计算 - 从诸多可以�?�择的轨迹中，�?�择�?个�?�最优�?�的路径
feval=[];
for id=1:length(evalDB(:,1))  % 遍历各个可运行的路径，分别计算其评价得分
    feval = [feval;evalParam(1:3)*evalDB(id,3:5)']; %根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得�?
end
evalDB = [evalDB feval]; % �?后一组；加最后一列，每一组�?�度的最终得�?
 
[maxv,ind] = max(feval);% 4)选取评分�?高的参数 对应分数返回�? maxv  对应下标返回�? ind
u = evalDB(ind,1:2)';% 返回�?优参数的速度、角速度  

%% 评价函数 内部负责产生可用轨迹   ok
% 输入参数 ：当前状态�?�参数允许范围（窗口）�?�目标点、障碍物位置、障碍物半径、评价函数的参数
%  Vr保存�?机器人当前状态可达到�? �?小最大的速度与角速度   model保存�?机器人的�?些�?�能参数，如该函数中使用�? 速度和角速度的分辨率
% 返回参数�? （返回一堆可以行进的轨迹～这些轨迹还�?进行评价函数的筛选，从�?�得到最终的前进路径�?
%           evalDB N*5  每行�?组可用参�? 分别�? 速度、角速度、航向得分�?�距离得分�?��?�度得分
%           trajDB      �?5行一条轨�? 每条轨迹包含 前向预测时间/dt + 1 = 31 个轨迹点（见生成轨迹函数�?
function [evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam)
evalDB = [];
trajDB = [];
for vt = Vr(1):model(5):Vr(2)       %根据速度分辨率遍历所有可用�?�度�? �?小�?�度和最大�?�度 之间 速度分辨�? 递增 
    for ot=Vr(3):model(6):Vr(4)     %根据角度分辨率遍历所有可用角速度�? �?小角速度和最大角速度 之间 角度分辨�? 递增  
        % 轨迹推测; 得到 xt: 机器人向前运动后的预测位�?; traj: 当前时刻 �? 预测时刻之间的轨迹（由轨迹点组成�?
        [xt,traj] = GenerateTrajectory(x,vt,ot,evalParam(4));  %evalParam(4),前向模拟时间;
        % 各评价函数的计算
        heading = CalcHeadingEval(xt,goal); % 前项预测终点的航向得�?  偏差越小分数越高
        [dist,Flag] = CalcDistEval(xt,ob,R);    % 前项预测终点 距离�?近障碍物的间隙得�? 距离越远分数越高
        vel     = abs(vt);                  % 速度得分 速度越快分越�?
        
        stopDist = CalcBreakingDist(vel,model); % 制动距离的计�?
        if dist > stopDist && Flag == 0 % 如果可能撞到�?近的障碍�? 则舍弃此路径 （到�?近障碍物的距�? 大于 刹车距离 才取用）
            evalDB = [evalDB;[vt ot heading dist vel]];   % flag 是否会碰到障碍物的标�?
            trajDB = [trajDB;traj];   % �?5�? �?条轨�?  
        end
    end
end

%% 归一化处�?     ok
% 每一条轨迹的单项得分除以本项�?有分数和
function EvalDB=NormalizeEval(EvalDB)
% 评价函数正则�?
if sum(EvalDB(:,3))~= 0  % 航向得分
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));  %矩阵的数�?  单列矩阵的每元素分别除以本列�?有数据的�?
end
if sum(EvalDB(:,4))~= 0  % 距离得分
    EvalDB(:,4) = EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~= 0  % 速度得分
    EvalDB(:,5) = EvalDB(:,5)/sum(EvalDB(:,5));
end

%% 单条轨迹生成、轨迹推演函�?.  ok
% 输入参数�? 当前状�?��?�vt当前速度、ot角�?�度、evaldt 前向模拟时间、机器人模型参数（没用到�?
% 返回参数;   返回 预测的x和到达该x�?经过的若干点 （将后�?�依次连线，就可得到�?条预测的轨迹�?
%           x   : 机器人模拟时间内向前运动 预测的终点位�?(状�??); 
%           traj: 当前时刻 �? 预测时刻之间 过程中的位姿记录（状态记录） 当前模拟的轨�?  
%                  轨迹点的个数�? evaldt / dt + 1 = 3.0 / 0.1 + 1 = 31         
function [x,traj] = GenerateTrajectory(x,vt,ot,evaldt)
global dt;
time = 0;
u = [vt;ot];% 输入�?
traj = x;   % 机器人轨�?
while time <= evaldt   
    time = time+dt; % 时间更新
    x = f(x,u);     % 运动更新 前项模拟时间�? 速度、角速度恒定
    traj = [traj x]; % 每一列代表一个轨迹点 �?列一列的添加
end

%% 计算制动距离   ok
%根据运动学模型计算制动距�?, 也可以�?�虑成走�?段段圆弧的累�? �?化可以当�?段段小直线的累积
% 利用 当前速度和机器人可达到的加�?�度，计算其速度减到0�?走距�?  
function stopDist = CalcBreakingDist(vel,model)
global dt;
MD_ACC   = 3;% 加�?�度
stopDist=0;
while vel>0   %给定加�?�度的条件下 速度减到0�?走的距离
    stopDist = stopDist + vel*dt;% 制动距离的计�? 
    vel = vel - model(MD_ACC)*dt;% 
end

%% 障碍物距离评价函�?    ok
%（机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
% 输入参数：位姿�?�所有障碍物位置、障碍物半径
% 输出参数：当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离 如果大于设定的最大�?�则等于�?大�??
% 距离障碍物距离越近分数越�?
function [dist,Flag] = CalcDistEval(x,ob,R)
dist=100;    % 无障碍物的默认�??
for io = 1:length(ob(:,1))  
    disttmp = norm(ob(io,:)-x(1:2)')-R; % 位置x到某个障碍物中心的距�? - 障碍物半�?  ！！！有可能出现负�?�吗
    if disttmp <0   % 该位置会碰到障碍�?
        Flag = 1;
        break;
    else            % 碰不到障碍物
        Flag = 0;
    end
    
    if dist > disttmp   % 大于�?小�?? 则�?�择�?小�??
        dist = disttmp;
    end
end
 
% 障碍物距离评价限定一个最大�?�，如果不设定，�?旦一条轨迹没有障碍物，将太占比重
if dist >= 3*R %�?大分数限�?
    dist = 3*R;
end
 
%% heading的评价函数计�?   ok
% 输入参数：当前位置�?�目标位�?
% 输出参数：航向参数得�? = 180 - 偏差�?
% 当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 �?�?180�?
function heading = CalcHeadingEval(x,goal)
theta = toDegree(x(3));% 机器人朝�?
goalTheta = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点相对于机器人本身的方位 
% 下面�? targetTheta 也就�? 小车当前航向与目标点的差�? （正数）
if goalTheta > theta
    targetTheta = goalTheta-theta;% [deg]
else
    targetTheta = theta-goalTheta;% [deg]
end
 
heading = 180 - targetTheta;  

%% 计算动�?�窗�?        model - 速度加�?�度等基本参数�??  ok
% 返回 �?小�?�度 �?大�?�度 �?小角速度 �?大角速度速度
function Vr = CalcDynamicWindow(x,model)
V_SPD       = 4;%机器人�?�度
W_ANGLE_SPD = 5;%机器人角速度 
MD_MAX_V    = 1;%   �?高�?�度m/s]
MD_MAX_W    = 2;%   �?高旋转�?�度[rad/s]
MD_ACC      = 3;%   加�?�度[m/ss]
MD_VW       = 4;%   旋转加�?�度[rad/ss]
global dt;
% 车子速度的最大最小范�? 依次为：�?小�?�度 �?大�?�度 �?小角速度 �?大角速度速度
Vs=[0 model(MD_MAX_V) -model(MD_MAX_W) model(MD_MAX_W)];
 
% 根据当前速度以及加�?�度限制计算的动态窗�?  依次为：�?小�?�度 �?大�?�度 �?小角速度 �?大角速度速度
Vd = [x(V_SPD)-model(MD_ACC)*dt x(V_SPD)+model(MD_ACC)*dt ...
    x(W_ANGLE_SPD)-model(MD_VW)*dt x(W_ANGLE_SPD)+model(MD_VW)*dt];
 
% �?终的Dynamic Window
Vtmp = [Vs;Vd];  % 2 X 4矩阵    每一列依次为：最小�?�度 �?大�?�度 �?小角速度 �?大角速度速度
Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))]; % 设定的参�? �? 计算的�?�度 比较

%% Motion Model 根据当前状�?�推算下�?个控制周期（dt）的状�?��??    oh！坐标变换的计算原理�?
% u = [vt; wt];当前时刻的�?�度、角速度 x = 状�?�[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
function x = f(x, u)
global dt;
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
 
x= F*x+B*u;  % 为何这样计算，暂不明�?

% 弧度和角度之间的换算
%% degree to radian
function radian = toRadian(degree)
radian = degree/180*pi;
%% radian to degree
function degree = toDegree(radian)
degree = radian/pi*180;
%% END