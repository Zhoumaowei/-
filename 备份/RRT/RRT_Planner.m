function [plan_succeeded, NodeList] = RRT_Planner(Map, origin, destination, sampling_points, step_length_limit, show_tree_only, show_animation) 
MapSize = size(Map);

% color setting
white = [1,1,1];
black = [0,0,0];
green = [0,1,0];
yellow = [1,1,0];
red = [1,0,0];
blue = [0,0,1];
cyan = [0,1,1];
color_list = [white; black; green; yellow; red; blue; cyan];
colormap(color_list);

logical_map = logical(Map);
map = zeros(MapSize(1),MapSize(2));
map(logical_map) = 2;
map(~logical_map) = 1;

iteration_times = 0;

start_node = Node(origin, 'S');
goal_node = Node(destination, 'G');
start_node = start_node.SetIndex(1);
node_list = [start_node];
  
image(0.5,0.5,map);
grid on;
set(gca,'xtick',1:1:MapSize(1));
set(gca,'ytick',1:1:MapSize(1));
axis image;
hold on;
plot(origin(1),origin(2),'.','Color','g','MarkerSize',30);
plot(destination(1),destination(2),'.','Color','y','MarkerSize',30);
drawnow;

plan_succeeded = 0;

while true
    if(iteration_times >= sampling_points || plan_succeeded == 1) % if exceed the sample times or plan succeeded
        break;
    end

    rand_node = [MapSize(1)*rand(1,1), MapSize(2)*rand(1,1)];
    add2list_flag = 0;
    if(map(ceil(rand_node(2)), ceil(rand_node(1))) ~= 2) % if the random node on the free space
        list_size = size(node_list);
        min_dis = inf;
        closest_node_index = 0;
        for i = 1:list_size(1)  % traverse the whole node_list to find closest node
            dis = distance(rand_node, node_list(i).coordinate); 
            if(dis < min_dis)
                closest_node_index = i;
                min_dis = dis;
            end
        end
        
        if(min_dis > step_length_limit) % if the minimum distance is longer than step length limit
            New_Node = Find_Inter_Node(node_list(closest_node_index).coordinate, rand_node, step_length_limit); % we should find a intermediate node.
        else
            New_Node = Node(rand_node, 'N');
        end
        
        if(ObstacleCheck(New_Node.coordinate, node_list(closest_node_index).coordinate, map, 0.01) == 0)% if the edge between nodes don't collision with obstacle
            add2list_flag = 1;
            % set parent node
            New_Node = New_Node.SetParent(node_list(closest_node_index).coordinate, distance(New_Node.coordinate, node_list(closest_node_index).coordinate), closest_node_index);
        end
    
        if(add2list_flag == 1)
            iteration_times = iteration_times + 1;
            if(show_animation)
                if(~show_tree_only)
                    plot(New_Node.coordinate(1),New_Node.coordinate(2),'.','Color','r','MarkerSize',15);
                end
                plot([New_Node.coordinate(1), New_Node.parent_coordinate(1)], [New_Node.coordinate(2), New_Node.parent_coordinate(2)], 'b');
              drawnow;
            end
            list_size = size(node_list);
            New_Node = New_Node.SetIndex(list_size(1)+1);
            node_list = [node_list; New_Node];
            
            dis = distance(destination, New_Node.coordinate); 
            if(dis <= step_length_limit)
                if(ObstacleCheck(New_Node.coordinate, destination, map, 0.01) == 0)
                    plan_succeeded = 1;
                    goal_node = goal_node.SetIndex(size(node_list, 1)+1);
                    goal_node = goal_node.SetParent(node_list(closest_node_index).coordinate, distance(New_Node.coordinate, node_list(closest_node_index).coordinate), closest_node_index);
                    node_list = [node_list; goal_node];
                end
            end
        end
    end
end
    
if(~show_animation)
    for i = 0:(size(node_list, 1)-2)
        k = size(node_list, 1)-i;
        if(~show_tree_only)
            plot(node_list(k).coordinate(1),node_list(k).coordinate(2),'.','Color','r','MarkerSize',15);
        end
        plot([node_list(k).coordinate(1), node_list(k).parent_coordinate(1)], [node_list(k).coordinate(2), node_list(k).parent_coordinate(2)], 'b');
    end
end

if(plan_succeeded)
    k = goal_node.index;
    while(k~=1)
        plot([node_list(k).coordinate(1), node_list(k).parent_coordinate(1)], [node_list(k).coordinate(2), node_list(k).parent_coordinate(2)], 'm', 'LineWidth', 2);
        k = node_list(k).parent_index;
    end
    drawnow;
end

NodeList = node_list;
end

