function [obj_fun] = new_fun_ca(u_hat_own)


%Only enters here when groups is in imminent collision
obj_fun=sum(u_hat_own.^2);
