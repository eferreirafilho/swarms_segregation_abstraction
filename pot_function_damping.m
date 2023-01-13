%Function that calcuates the olfati-saber force considering that 
%the abstraction being treated is using a estimated mean

function [Vx_damping,Vy_damping]=pot_function_damping(mu_est_x,mu_est_y,mu_est_x_dot,mu_est_y_dot,rx,ry,vx,vy,r_alpha,h,eps_norm_sigma,group_number)

n_abs=size(rx,2);

%Preallocating Variables
norm_r(1:n_abs,1:n_abs)=0;
norm_sigma(1:n_abs,1:n_abs)=0;
aij(1:n_abs,1:n_abs)=0;
Vx_damping(1:n_abs,1:n_abs)=0;
Vy_damping(1:n_abs,1:n_abs)=0;

%POTENCIAL FUNCTION DAMPING (Must be calculated separedtly for each robot)
    
%DISTANCES
for i=1:n_abs
for j=1:n_abs
 
    if i==group_number
        rx(i)=mu_est_x;
        ry(i)=mu_est_y;
    end
    if j==group_number
        rx(j)=mu_est_x;
        ry(j)=mu_est_y;
    end
    
    %Eucledian distances
    norm_r(i,j)=sqrt((rx(i) - rx(j))*(rx(i) - rx(j)) + (ry(i) - ry(j))*(ry(i) - ry(j)));

    %Sigma norm
    norm_sigma(i,j)=(1/eps_norm_sigma)*((sqrt(1+eps_norm_sigma*(norm_r(i,j))^2))-1);

end
end
%DISTANCES END
for i=1:n_abs
for j=1:n_abs

    aij(i,j)=phz(norm_sigma(i,j)/r_alpha,h);

if(i==j)
    aij(i,j)=0;
end

end
end

for i=1:n_abs
for j=1:n_abs
     if i==group_number
        vx(i)=mu_est_x_dot;
        vy(i)=mu_est_y_dot;
    end
    if j==group_number
        vx(j)=mu_est_x_dot;
        vy(j)=mu_est_y_dot;
    end
    
    Vx_damping(i,j)=aij(i,j)*(vx(j)-vx(i));
    Vy_damping(i,j)=aij(i,j)*(vy(j)-vy(i));
end
end


end
  
 