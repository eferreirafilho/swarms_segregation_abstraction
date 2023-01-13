%Function that calcuates the olfati-saber force considering that 
%the abstraction being treated is using a estimated mean

function [potfnx,potfny]=pot_function(mu_est_x,mu_est_y,rx,ry,group_number,eps_norm_sigma,r,d_alpha,r_alpha,a,b,h,c)

%Parameters
%eps_norm_sigma=pot_param(1);
%r=pot_param(2);
%d_alpha=pot_param(3);
%r_alpha=pot_param(4);
%a=pot_param(5);
%b=pot_param(6);
%h=pot_param(7);
%c=pot_param(8);

n_abs=size(rx,2);

%Preallocating Variables
norm_r(n_abs,n_abs)=0;
norm_sigma(1:n_abs)=0;
potfnx(1:n_abs)=0;
potfny(1:n_abs)=0;

%DISTANCES
for i=1:n_abs
 
    %Eucledian distances from every abs to the one being treated
    norm_r(i)=sqrt((rx(i) - mu_est_x)*(rx(i) - mu_est_x) + (ry(i) - mu_est_y)*(ry(i) - mu_est_y));
    if i==group_number
        norm_r(i)=0;
    end
end

for i=1:n_abs
    %Sigma norm
    norm_sigma(i)=(1/eps_norm_sigma)*((sqrt(1+eps_norm_sigma*(norm_r(i))^2))-1);
end

%POTENCIAL FUNCTION (Must be calculated separedtly for each robot)
for i=1:n_abs
    potfnx(i)=0.5*((a+b)*((norm_sigma(i)-d_alpha+c)/sqrt(1+(norm_sigma(i)-d_alpha+c)^2)) +(a-b))*phz((norm_sigma(i)/r_alpha),h)*((rx(i) - mu_est_x)/(1+eps_norm_sigma*((norm_sigma(i)))));
    potfny(i)=0.5*((a+b)*((norm_sigma(i)-d_alpha+c)/sqrt(1+(norm_sigma(i)-d_alpha+c)^2)) +(a-b))*phz((norm_sigma(i)/r_alpha),h)*((ry(i) - mu_est_y)/(1+eps_norm_sigma*((norm_sigma(i)))));

    if i==group_number
         potfnx(i)=0;
         potfny(i)=0;
    end


end
  
 