clc
clear
close all
tic
%% ����
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% ����ɽ���ͼ
mapRange = [100,100,100];             
[X,Y,Z] = defMap(mapRange);

%% ������ά�ռ�����·���滮����Ƭ�ṹ��

% ������Ƭ�ṹ�壬����ά�ռ���Ƭ�ֲ㴦��
sliceNum = 9;
slice = struct;
slice.allowedPos = [];                % ÿƬ��Ƭ���������դ��
slice.par = [];                       % ÿһ����Ƭ������һ����Ƭ�Ĳ�������Ϣ�ص�
slice = repmat(slice,sliceNum,1);

% ���ÿһ����Ƭ��������ʵ�դ��
for i = 1:sliceNum
    if i == 1
        slice(i).allowedPos = startPos;
    elseif i == sliceNum
        slice(i).allowedPos = goalPos;
    else
        h = (i-1)*10;  % ��Ƭ�߶�
        for x = X(1,1):10:X(1,end)
            for y = Y(1,1):10:Y(end,1)
                if h > Z(x,y)
                    slice(i).allowedPos(end+1,:) = [x,y,h];
                end
            end
        end
    end
end

% ��ʼ����Ϣ�غ�����ֵ
for i = 1:sliceNum-1%����ѭ����
    for j = 1:size(slice(i).allowedPos,1)
        pathNum = size(slice(i+1).allowedPos,1);
        slice(i).par(j).tau = ones(pathNum,1);
        deltaX = slice(i+1).allowedPos(:,1) - slice(i).allowedPos(j,1);
        deltaY = slice(i+1).allowedPos(:,2) - slice(i).allowedPos(j,2);
        deltaZ = slice(i+1).allowedPos(:,3) - slice(i).allowedPos(j,3);
        slice(i).par(j).eta = 100./sqrt(deltaX.^2 + deltaY.^2 + deltaZ.^2);
    end
end

%% ������Ⱥ�ṹ��
% ��Ⱥ��ض���
m = 100;                             % ��������
alpha = 10;                          % ��Ϣ����Ҫ�̶�����
beta = 1;                            % ��������
rho = 0.1;                           % ��Ϣ�ػӷ�����
Q = 1;                               % ����
iter_max = 15;                      % ����������

% ������Ⱥ�ṹ��
antColony = struct;
antColony.pos= [];
antColony.path = [];
antColony.fitness = [];
antColony.Best.pos = [];
antColony.Best.path = [];
antColony.Best.fitness = inf;
antColony = repmat(antColony,m,1);

%�������ϳ�ʼ��
GlobalBest.fitness=inf;

%% ����Ѱ��
for  iter = 1:iter_max
 
    % ��ʼ����Ⱥ·���仯��Ϣ�ؽṹ��
    deltaTau = struct;
    deltaTau.start = [0,0,0];
    deltaTau.target = [0,0,0];
    deltaTau.delat_tau = (0);
    
    % �������·��ѡ��
    for i = 1:m
        
        % ����ʼλ�ú�Ŀ��λ�ô������Ⱥ�ṹ����
        antColony(i).pos(1,:) = startPos;
        antColony(i).pos(sliceNum,:) = goalPos;
        nowPos = startPos;
        idx = 1;
        
        % �����Ƭ��դ��ѡ��
        for j = 1:sliceNum-2            
            %������һ���ڵ�ķ��ʸ���
            P = slice(j).par(idx).tau .^alpha .* slice(j).par(idx).eta .^ beta;
            P = P/sum(P);
            
            % ���̷�ѡ����һ�����ʽڵ�
            Pc = cumsum(P);
            Pc = [0; Pc];
            randnum = rand;
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    targetPos = slice(j+1).allowedPos(k,:);
                    break
                end
            end
            
            % �������ϵĵ�ǰλ�ú�����
            antColony(i).pos(j+1,:) = targetPos;
            nowPos = targetPos;
            idx = k; 
        end
        
        % ����ÿһ����Ƭ��դ��㣬���ò�ֵ��ϵõ���ά·��
        [flag,fitness,path] = calFitness(startPos, goalPos,X,Y,Z, antColony(i).pos);
        antColony(i).path = path;
        
        % �ж�·��������
        if flag == 0
            % ����ײ
            antColony(i).fitness = fitness;

            % ���µ�ֻ���ϵ�����?
            if antColony(i).fitness < antColony(i).Best.fitness
                antColony(i).Best.pos = antColony(i).pos ;
                antColony(i).Best.path = antColony(i).path ;
                antColony(i).Best.fitness = antColony(i).fitness ;
            end
            
            % ȫ������
            if antColony(i).Best.fitness < GlobalBest.fitness
                GlobalBest = antColony(i).Best;
            end
            
            % ͳ�ơ����¾���ĳ��·������Ϣ������?
            for j = 1:sliceNum-1
                % ��ж��Ƿ��Ѿ�����deltaTau
                [~,idx1] = ismember(antColony(i).pos(j,:), deltaTau.start, 'rows');
                [~,idx2] =  ismember(antColony(i).pos(j+1,:), deltaTau.target, 'rows');
                if idx1 == idx2 && idx1 ~= 0
                    deltaTau.delat_tau(idx1)  = deltaTau.delat_tau(idx1) + Q / fitness;
                else
                    deltaTau.start(end+1,:) = antColony(i).pos(j,:);
                    deltaTau.target(end+1,:)  = antColony(i).pos(j+1,:);
                    deltaTau.delat_tau(end+1,1)  = Q / fitness;
                end
            end    
            
        else
             % ��ײ
             antColony(i).fitness = 1000* fitness;
        end
    end
    
    % ���ǻӷ����ӣ�������Ϣ��
    for i = 1:sliceNum-1
        for j = 1:size(slice(i).allowedPos,1)
            start = slice(i).allowedPos(j,:);
            for k = 1:size(slice(i).par(j).tau,1)
                target = slice(i+1).allowedPos(k,:);
                [~,idx1] = ismember(start, deltaTau.start, 'rows');
                [~,idx2] = ismember(target, deltaTau.target, 'rows');
                if idx1 == idx2 && idx1~=0
                    slice(i).par(j).tau(k,1) = (1-rho)* slice(i).par(j).tau(k,1) +...
                        deltaTau.delat_tau(idx1,1);
                else
                    slice(i).par(j).tau(k,1) = (1-rho)* slice(i).par(j).tau(k,1);
                end
            end
        end
    end
    
    % ÿһ������fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % ��ʾÿһ����Ϣ
    disp(['��' num2str(iter) '��:' '������Ӧ�� = ' num2str(fitness_beat_iters(iter))]);
    
    % ��ͼ
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);
end

%% ���չʾ
% ������С��Ӧ��
fitness_best = norm(startPos - goalPos);
disp([ '����������Ӧ�� = ' num2str(fitness_best)]);

% ����Ӧ�ȵ���ͼ
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('��������');
ylabel('������Ӧ��');
toc
