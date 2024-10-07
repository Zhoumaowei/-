%load('Map1.mat');   
%load('Map2.mat');
 load('Map3.mat');
 %load('field.mat')
%load('field.mat')
start_node = [0, 0];    % coordinate of the start node
dest_node  = [100, 50]; % coordinate of the destination node
figure(1);
clf;

show_tree_only = true;
show_animation = true;  % set this true to show the animation
sampling_points = inf;
step_length_limit = 3;

tic;
%[plan_succeeded, NodeList] = RRT_Planner(map, start_node, dest_node, sampling_points, step_length_limit, show_tree_only, show_animation);
[plan_succeeded, NodeList1, NodeList2] = RRT2_Planner(map, start_node, dest_node, sampling_points, step_length_limit, show_tree_only, show_animation);
toc;

if(plan_succeeded)
    disp('plan succeeded! ');
else
    disp('plan failed!');
end

