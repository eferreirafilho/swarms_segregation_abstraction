%Function that receives robots positions, size and delta collision and
%returns which robots are in imminent collision


function [index_r,ca_norm,norm_r]=robots_in_imminent_collision(rrx_ca,rry_ca,radius,delta_collision,M)

    n_abs=size(M,2);
    N=sum(M);
    %n_robots=size(rrx,1);

    
    %Collision Graph
    %Distance between robots
    for i=1:N
    for j=1:N       
    norm_r(i,j)=sqrt((rrx_ca(i) - rrx_ca(j))*(rrx_ca(i) - rrx_ca(j)) + (rry_ca(i) - rry_ca(j))*(rry_ca(i) - rry_ca(j)));
          if i==j
              norm_r(i,j)=NaN;
          end
    end
    end
    
    %Matrix with robots that are close enought
    for i=1:N
    for j=1:N  
          if(norm_r(i,j)<(delta_collision) && i~=j)
              ca_norm(i,j)=1;
          else
              ca_norm(i,j)=0;
         end
    end
    end

    %Which robots are in collision
    sum_ca_norm = sum(ca_norm);
    for i=1:N
    if sum_ca_norm(i)>=1
        index_ca(i)=1;
    else
    index_ca(i)=0;
    end
    end
    %for k=1:n_abs
    %    index_r(:,k)=index_ca(M(k)*k-(M(k)-1):M(k)*k);
    %end
    
    %Transform Variables
    cont=1;
    for i=1:n_abs
        index_r(1:M(i),i)=index_ca(cont:(cont+M(i)-1));
        cont=cont+M(i);
    end
    
    
    
    
    
    