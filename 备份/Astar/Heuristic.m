function h = Heuristic(node1, node2, scale)
    %% Manhattan Distance
    h = scale*(abs(node1(1) - node2(1)) + abs(node1(2) - node2(2)));

    %% Chebyshev Distance
    %h = scale*max([node1(1) - node2(1), node1(2) - node2(2)]);
    
    %% Euclidean Distance
    %h = scale*sqrt((node1(1) - node2(1))^2 + (node1(2) - node2(2))^2);   
end

