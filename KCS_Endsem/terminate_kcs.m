function [value,isterminal,direction] = terminate_kcs(t,x)

global terminate_flag

if terminate_flag == 1
    value = 1;
    isterminal = 1;
    direction = 0;
else
    value = 0;
    isterminal = 0;
    direction = 0;
end