classdef Node
    properties
        type;   % node type
        coordinate; % coordinate of node
        index;      % the index of current nodes in the node_list
        parent_coordinate; %  coordinate of parent nodes
        parent_index; % the index of parent nodes in the node_list
        cost;   % the cost of edges
    end
    
    methods
        %% Construct function
        function N = Node(Coordinate, Type)
            N.type = Type;
            N.coordinate = Coordinate;
        end
        
        %% SetIndex function
        function N = SetIndex(N, Index)
            N.index = Index;
        end
        
        %% SetParent function
        function N = SetParent(N, parent_node_coordinate, cost, index)
            N.parent_coordinate = parent_node_coordinate;
            N.parent_index = index;
            N.cost = cost;
        end
    end
end