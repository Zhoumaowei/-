% �?5讲：人工蜂群算法
% 作�?�： Ally
% 日期�? 2021/08/29
clc
clear
close all

%% 三维路径规划模型
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% 随机定义山峰地图
mapRange = [100,100,100];              % 地图长�?�宽、高范围
[X,Y,Z] = defMap(mapRange);

% 位置界限
posBound = [[0,0,0]',mapRange'];

%% 初始参数设置
Ns = 15;           % 蜜蜂总数
Ne = Ns/2;          % 采蜜蜂数�? = 观察蜂数�? = 蜜源数量
iterMax = 10;      % �?大迭代次�?
limit = 5;          % 陷入�?部最优的次数判断阈�??
pointNum = 3;       % 每一个蜜源包含三个位置点

%% 初始化蜜源位�?
% 初始化一个空的蜜源结构体
nectarSource = struct;
nectarSource.pos = [];
nectarSource.fitness = [];
nectarSource.path = [];
nectarSource.limitNum = 1;
nectarSource = repmat(nectarSource,Ne,1);

% 初始化每�?代的�?优蜜�?
GlobalBest.fitness = inf;

% 第一代的蜜源位置初始�?
for i = 1:Ne
    % 蜜源按照正�?�分布随机生�?
    nectarSource(i).pos.x = unifrnd(posBound(1,1),posBound(1,2),1,pointNum);
    nectarSource(i).pos.y = unifrnd(posBound(2,1),posBound(2,2),1,pointNum);
    nectarSource(i).pos.z = unifrnd(posBound(3,1),posBound(3,2),1,pointNum);
    
    % 适应�?
    [flag,fitness,nectarSource(i).path] = calFitness(startPos, goalPos,X,Y,Z, nectarSource(i).pos);
    
    % 碰撞�?测判�?
    if flag == 1
        % 若flag=1，表明此路径将与障碍物相交，则增大�?�应度�??
        nectarSource(i).fitness = 1000*fitness;
    else
        % 否则，表明可以�?�择此路�?
        nectarSource(i).fitness = fitness;
    end
    
    % 更新全局�?�?
    if nectarSource(i).fitness < GlobalBest.fitness
        GlobalBest = nectarSource(i);
    end
end

% 初始化每�?代的�?优�?�应度，用于画�?�应度迭代图
fitness_beat_iters = zeros(iterMax,1);

%% 循环迭代
for iter = 1:iterMax
    % 第一步：采蜜蜂在蜜源位置附近寻找新的蜜源
    for i = 1:Ne
        
        % 生成k�?
        while true
            k = randi([1,Ne]);
            if k ~=  i
                break
            end
        end
        
        % 根据公式更新蜜源位置
        pos_new.x =  nectarSource(i).pos.x + rand * (nectarSource(k).pos.x - nectarSource(i).pos.x);
        pos_new.y =  nectarSource(i).pos.y + rand * (nectarSource(k).pos.y - nectarSource(i).pos.y);
        pos_new.z =  nectarSource(i).pos.z + rand * (nectarSource(k).pos.z - nectarSource(i).pos.z);
        
        % 判断是否位于位置界限以内
        pos_new.x = max(pos_new.x, posBound(1,1));
        pos_new.x = min(pos_new.x, posBound(1,2));
        pos_new.y = max(pos_new.y, posBound(2,1));
        pos_new.y = min(pos_new.y, posBound(2,2));
        pos_new.z = max(pos_new.z, posBound(3,1));
        pos_new.z = min(pos_new.z, posBound(3,2));

        % 计算适应�?
        [flag,fitness_new,path_new] = calFitness(startPos, goalPos,X,Y,Z, pos_new);
        
        % 碰撞�?测判�?
        if flag == 1
            % 若flag=1，表明此路径将与障碍物相交，则增大�?�应度�??
            fitness_new = 1000*fitness_new;
        end
        
        % 判断新蜜源与之前蜜源的�?�应度大�?
        if fitness_new < nectarSource(i).fitness
            nectarSource(i).pos = pos_new;
            nectarSource(i).fitness = fitness_new;
            nectarSource(i).path = path_new;
            nectarSource(i).limitNum = 1;
        else
            nectarSource(i).limitNum = nectarSource(i).limitNum + 1;
        end
        
        % 更新全局�?�?
        if nectarSource(i).fitness < GlobalBest.fitness
            GlobalBest = nectarSource(i);
        end
    end
    
    % 第二步：采蜜蜂回到蜂巢跳摆尾舞与跟随蜂共享蜜源信�?
    % 每一只跟随蜂根据蜜源适应度，以一定概率�?�择具体的蜜源，并在周围采蜜
    for i = 1:Ne
        % 计算每一个蜜源的适应度占据的概率
        fitness_all = 1./[nectarSource.fitness]';
        P = fitness_all/sum(fitness_all);
        
        % 轮盘赌法选择蜜源
        Pc = cumsum(P);
        Pc = [0; Pc];
        randNum = rand;
        for j = 1:length(Pc)-1
            if randNum > Pc(j) && randNum < Pc(j+1)
                k = j;
                break
            end
        end
           
        % 根据公式更新蜜源位置
        pos_new.x =  nectarSource(i).pos.x + rand * (nectarSource(i).pos.x - nectarSource(k).pos.x);
        pos_new.y =  nectarSource(i).pos.y + rand * (nectarSource(i).pos.y - nectarSource(k).pos.y);
        pos_new.z =  nectarSource(i).pos.z + rand * (nectarSource(i).pos.z - nectarSource(k).pos.z);
        
        % 判断是否位于位置界限以内
        pos_new.x = max(pos_new.x, posBound(1,1));
        pos_new.x = min(pos_new.x, posBound(1,2));
        pos_new.y = max(pos_new.y, posBound(2,1));
        pos_new.y = min(pos_new.y, posBound(2,2));
        pos_new.z = max(pos_new.z, posBound(3,1));
        pos_new.z = min(pos_new.z, posBound(3,2));
        
        % 计算适应�?
        [flag,fitness_new,path_new] = calFitness(startPos, goalPos,X,Y,Z, pos_new);
        
        % 碰撞�?测判�?
        if flag == 1
            % 若flag=1，表明此路径将与障碍物相交，则增大�?�应度�??
            fitness_new = 1000*fitness_new;
        end
        
        % 判断新蜜源与之前蜜源的�?�应度大�?
        if fitness_new < nectarSource(i).fitness
            nectarSource(i).pos = pos_new;
            nectarSource(i).fitness = fitness_new;
            nectarSource(i).path = path_new;
            nectarSource(i).limitNum = 1;
        else
            nectarSource(i).limitNum = nectarSource(i).limitNum + 1;
        end
        
        % 更新全局�?�?
        if nectarSource(i).fitness < GlobalBest.fitness
            GlobalBest = nectarSource(i);
        end
        
        % 第三步：若此处的蜜源陷入�?部最优，则调用侦查蜂生成随机位置
        if nectarSource(i).limitNum > limit &&...
                nectarSource(i).fitness > GlobalBest.fitness
            
            % 更新位置
            nectarSource(i).pos.x = nectarSource(i).pos.x + (2*rand-1) * 0.5*mapRange(1);
            nectarSource(i).pos.y = nectarSource(i).pos.y + (2*rand-1) * 0.5*mapRange(2);
            nectarSource(i).pos.z = nectarSource(i).pos.z + (2*rand-1) * 0.5*mapRange(3);
            
            % 判断是否位于位置界限以内
            nectarSource(i).pos.x = max(nectarSource(i).pos.x, posBound(1,1));
            nectarSource(i).pos.x = min(nectarSource(i).pos.x, posBound(1,2));
            nectarSource(i).pos.y = max(nectarSource(i).pos.y, posBound(2,1));
            nectarSource(i).pos.y = min(nectarSource(i).pos.y, posBound(2,2));
            nectarSource(i).pos.z = max(nectarSource(i).pos.z, posBound(3,1));
            nectarSource(i).pos.z = min(nectarSource(i).pos.z, posBound(3,2));
            
            
            % 计算适应�?
            [flag,fitness_new,path_new] = calFitness(startPos, goalPos,X,Y,Z, pos_new);
            
            % 碰撞�?测判�?
            if flag == 1
                % 若flag=1，表明此路径将与障碍物相交，则增大�?�应度�??
                nectarSource(i).fitness = 1000*fitness_new;
            end
            
            % 更新全局�?�?
            if nectarSource(i).fitness < GlobalBest.fitness
                GlobalBest = nectarSource(i);
            end
            
            % 重置limitNum = 1
            nectarSource(i).limitNum = 1;
        end
        
    end
    
    % 把每�?代的�?优粒子赋值给fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['�?' num2str(iter) '�?:' '�?优�?�应�? = ' num2str(fitness_beat_iters(iter))]);
    
    % 画图
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);

end

% 理论�?小�?�应度：直线距离
fitness_best = norm(startPos - goalPos);
disp([ '理论�?优�?�应�? = ' num2str(fitness_best)]);

% 画�?�应度迭代图
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('迭代次数');
ylabel('�?优�?�应�?');
