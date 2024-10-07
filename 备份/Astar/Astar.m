%% functionname: function description

% display setting:
% grid type	 | grid color | identifier
% -----------------------------------
% free space | white      | 1
% obstacle   | black      | 2
% origin     | green      | 3
% destination| yellow	  | 4
% finished	 | red		  | 5
% unfinished | blue		  | 6
% path		 | cyan		  | 7

function [path, iteration_times] = Astar(Map, origin, destination)
% record the iteration times
iteration_times = 0;

%set the scale factor of Heuristic function
Heuristic_scale = 1;

% identifier setting
Obstacle = 2;
Origin = 3;
Destination = 4;
Finished = 5;
Unfinished = 6;
Path = 7;

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

% listes initialize
MapSize = size(Map);

% create map
logical_map = logical(Map);
map = zeros(MapSize(1),MapSize(2));
map(logical_map) = 2;
map(~logical_map) = 1;

% create node_g_list
node_g_list = Inf(MapSize(1), MapSize(2));
node_g_list(origin(1), origin(2)) = 0;	% set the node_cost of the origin node zero

% create node_f_list
node_f_list = Inf(MapSize(1), MapSize(2));
node_f_list(origin(1), origin(2)) = Heuristic(origin, destination, Heuristic_scale);

% create parent_list
parent_list = zeros(MapSize(1), MapSize(2));

destination_index = sub2ind(MapSize, destination(1), destination(2));
origin_index = sub2ind(MapSize, origin(1), origin(2));

Open_list = [origin_index];

plan_succeeded = false;

while true
    iteration_times = iteration_times+1;
    map(origin(1), origin(2)) = Origin;
    map(destination(1), destination(2)) = Destination;
    
% uncomment this part to show the animation, but it will spend more time during algorithm running.    
% 	image(0.5,0.5,map);
% 	grid on;
% 	set(gca,'xtick',1:1:MapSize(1));
% 	set(gca,'ytick',1:1:MapSize(1));
% 	axis image;
% 	drawnow;
    
	[min_node_cost, current_node_index] = min(node_f_list(:));
	if(min_node_cost == inf || current_node_index == destination_index)
		plan_succeeded = true;
		break;
    end
    node_f_list(current_node_index) = inf;
    map(current_node_index) = Finished;
	[x,y] = ind2sub(MapSize, current_node_index);
    for k = 0:3	% four direction
		if(k == 0)
		    adjacent_node = [x-1,y];
		elseif (k == 1)
			adjacent_node = [x+1,y];
		elseif (k == 2)
			adjacent_node = [x,y-1];
		elseif(k == 3)
			adjacent_node = [x,y+1];
		end
		
		if((adjacent_node(1) > 0 && adjacent_node(1) <= MapSize(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= MapSize(2))) % make sure the adjacent_node don't exceeds the map
            if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle && map(adjacent_node(1),adjacent_node(2)) ~= Finished)
                if(node_g_list(adjacent_node(1),adjacent_node(2)) > min_node_cost + 1 )
                    node_g_list(adjacent_node(1),adjacent_node(2)) = node_g_list(current_node_index) + 1;                 
                    node_f_list(adjacent_node(1),adjacent_node(2)) = node_g_list(adjacent_node(1),adjacent_node(2)) + Heuristic(adjacent_node, destination, Heuristic_scale);                  
                    %uncomment this line to change Astar to Greedy algorithm
                    %node_f_list(adjacent_node(1),adjacent_node(2)) = Heuristic(adjacent_node, destination, Heuristic_scale);
                    
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        parent_list(adjacent_node(1),adjacent_node(2)) = 0;	% Set the parent 0 if adjacent_node is the origin.
                    else
                        parent_list(adjacent_node(1),adjacent_node(2)) = current_node_index;	%Set the parent current_node_index
                    end
                    if(map(adjacent_node(1),adjacent_node(2)) ~= Unfinished)
                        map(adjacent_node(1),adjacent_node(2)) = Unfinished;	% Mark the adjacent_node as unfinished
                    end
                end
            end
        end
	end
end

if(plan_succeeded)
	path = [];
	node = destination_index;
	while(parent_list(node) ~= 0)
		path = [parent_list(node), path];
		node = parent_list(node);
    end
    
	for k = 2:size(path,2)
		map(path(k)) = 7;
		image(0.5,0.5,map);
		grid on;
        set(gca,'xtick',1:1:MapSize(1));
        set(gca,'ytick',1:1:MapSize(1));
		axis image;
		drawnow;
	end
else
	path = [];
end
end
