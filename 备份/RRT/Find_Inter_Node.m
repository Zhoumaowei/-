function [Node_I] = Find_Inter_Node (start, target, step_length)
    ky = target(2)-start(2);
    kx = target(1)-start(1);
    theta = atan2(ky, kx);
    coordinate = [start(1)+cos(theta)*step_length, start(2)+sin(theta)*step_length];
    Node_I = Node(coordinate, 'N');
end

