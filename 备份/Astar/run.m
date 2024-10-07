%load('Map1.mat');   
% import the existed map
load('field.mat')
start_node = [1, 1];    % coordinate of the start node
dest_node  = [6, 7]; % coordinate of the destination node

% load('Map2.mat'); %uncomment this line to use the 100*100 map instead of 20*20 map
% map = zeros(100); % uncomment this line to replace map with a 100*100 zero matrix

figure(1);
[path, iteration_times] = Astar(map, start_node, dest_node);

if(size(path,2) ~= 0)
    display(['A* plan succeeded! ','iteration times: ',num2str(iteration_times), ' path length: ', num2str(size(path,2))]);
else
    display('A* plan failed!');
end

% uncomment this part to compare with Dijkstar
figure(2);
[path, iteration_times] = Dstar(map, start_node, dest_node);

if(size(path,2) ~= 0)
    display(['D* plan succeeded! ','iteration times: ',num2str(iteration_times), ' path length: ', num2str(size(path,2))]);
else
    display('D* plan failed!');
end
