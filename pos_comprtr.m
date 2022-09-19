% function to compare

function out = pos_comprtr(uav_states,MS_states)
    del_x = uav_states(1) - MS_states(1);
    del_y = uav_states(2) - MS_states(2);
    del_z = uav_states(3) - MS_states(3);
    out = [del_x;del_y;del_z];
end