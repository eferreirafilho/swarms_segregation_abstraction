%Function to 

function [c,ceq]= new_con_ca_groups(u_hat,ca_norm,d_phi_null,UUx,UUy,radius,rrx,rry,vvx,vvy,groups_idx,delta_collision,N,M,wg)


n_abs=size(rrx,2);
%n_robots=size(rrx,1);


%Transform Variables
cont=1;
for i=1:n_abs
    rrx_ca(cont:(cont+M(i)-1),1)=rrx(1:M(i),i);
    rry_ca(cont:(cont+M(i)-1),1)=rry(1:M(i),i);
    vvx_ca(cont:(cont+M(i)-1),1)=vvx(1:M(i),i);
    vvy_ca(cont:(cont+M(i)-1),1)=vvy(1:M(i),i);
    UUx_ca(cont:(cont+M(i)-1),1)=UUx(1:M(i),i);
    UUy_ca(cont:(cont+M(i)-1),1)=UUy(1:M(i),i);
    cont=cont+M(i);
end

%u_hat has all groups stacked

%Put zeros in groups that are not being treated
cont=1;
for i=1:n_abs
    if ~ismember(i,groups_idx) %Group i not being treated
        %u_hat(i*n_robots*2-n_robots*2+1:2*n_robots*i)=0;
        u_hat(cont:(cont+2*M(i)-1))=0;
        cont=cont+2*M(i);
    end   
end

%d_phi_null_v -> null space control inputs X and Y

%Stacked dphi of unbalanced groups
cont=1;
for j=1:n_abs
    d_phi_null_sta(cont:(cont+2*M(j)-1),cont:(cont+2*M(j)-1))=d_phi_null{j};
    cont=cont+2*M(j);
end

d_phi_null_v=d_phi_null_sta*u_hat';
    
% % %     %Transforming variables
% % %     cont=1;
% % %     for i=1:n_abs
% % %         UUx_ca(cont:(cont+M(i)-1),1)=UUx(1:M(i),i);
% % %         UUy_ca(cont:(cont+M(i)-1),1)=UUy(1:M(i),i);
% % %         cont=cont+M(i);
% % %     end
% % %  

d_phi_null_x_con(1:N)=0;
d_phi_null_y_con(1:N)=0;
%Transforming null space to x and y
cont_c=1;
for i=1:2:2*N
    d_phi_null_x_con(cont_c)=d_phi_null_v(i);
    cont_c=cont_c+1;
end
cont_c=1;
for i=2:2:2*N
    d_phi_null_y_con(cont_c)=d_phi_null_v(i);
    cont_c=cont_c+1;
end
        
%Adding the CA inputs to old inputs
UUx_new_ca=UUx_ca+d_phi_null_x_con;
UUy_new_ca=UUy_ca+d_phi_null_y_con;
 

%% Add a constraint for each pair of robots in imminent collision

%Which are the pairs?

%Must belong to the groups being treated
%Put zeros in ca_norm of groups not being treated

%If robot i has a collision with a group not being treated, zero all his
%group in ca_norm
for i=1:N
    for j=1:N
        if ca_norm(i,j)==1
            %If robot i or robot j not in the groups being treated
            if ~ismember(wg(i),groups_idx)% || ~ismember(j,groups_idx)
                %Put zeros in ca_norm of groups not being treated  
                ca_norm(i,j)=0;
                ca_norm(j,i)=0;             
            end
        end
    end
end


%% Add collisions for each pair of robots
ii=1; %Count Constraints
c=[];  %if there is not a 
for i=1:N
    for j=1:N
        if (ca_norm(i,j)==1 && i~=j)
            
           
            %Adding one new constraint
             qdot_vec=[(vvx_ca(i) - vvx_ca(j));(vvy_ca(i) - vvy_ca(j))];
            qdot_vec_k=[(vvx_ca(i));(vvy_ca(i))];
            qdot_vec_l=[(vvx_ca(j));(vvy_ca(j))];
            q_vec=[(rrx_ca(i) - rrx_ca(j));(rry_ca(i) - rry_ca(j))];
            
            
           %If condition 2 is satisfied
           if((qdot_vec')*(q_vec)<0 || (-qdot_vec')*(-q_vec)<0)%SECOND CONDITION (As in Master's Dissertation Chapter)
           
            
            %Now, the new input (UUx_new) must satisfie the constraints
            q_vec_norm=sqrt((rrx_ca(i) - rrx_ca(j))^2 + (rry_ca(i) - rry_ca(j))^2);
            %disp(i)
            %disp(j)
%             figure(1)
%             hold on
%             drawnow
%             circle(rrx_ca(i),rry_ca(i),delta_collision-radius,[0 0 0])
%             circle(rrx_ca(j),rry_ca(j),delta_collision-radius,[0 0 0])
%             %Transforming variables to draw
%             for k=1:n_abs
%                 UUx_ca_plot(n_robots*k-(n_robots-1):n_robots*k)=UUx(:,k);
%                 UUy_ca_plot(n_robots*k-(n_robots-1):n_robots*k)=UUy(:,k);
%             end
%            quiver(rrx_ca(i),rry_ca(i),UUx_ca_plot(i),UUy_ca_plot(i),0.5,'color',[0 0 0])
%            quiver(rrx_ca(j),rry_ca(j),UUx_ca_plot(j),UUy_ca_plot(j),0.5,'color',[0 0 0])
            
            for iii=1:2*N
                  u_hat_old(iii,1)=0;
            end
            u_hat_old(2*i-1)=UUx_new_ca(i);
            u_hat_old(2*i)=UUy_new_ca(i);
            u_hat_old(2*j-1)=UUx_new_ca(j);
            u_hat_old(2*j)=UUy_new_ca(j);
            
            
            new_u_hat_vec_i= [(u_hat_old(2*i-1)); (u_hat_old(2*i))]; 
            new_u_hat_vec_j= [(u_hat_old(2*j-1)); (u_hat_old(2*j))];
            %%%%SUBJECT TO (1)  
            transpose_qdot_vec_k=transpose(qdot_vec_k);
            cd1_eq_left_side= transpose(new_u_hat_vec_i)*(q_vec/q_vec_norm);
            %%%%SUBJECT TO (2)
            transpose_qdot_vec_l=transpose(qdot_vec_l);
            cd2_eq_left_side= transpose(new_u_hat_vec_j)*(-q_vec/q_vec_norm);
            %for small_delta=radius-0.5*radius:0.2:delta_collision
            for small_delta=radius

                %%%%SUBJECT TO (1)
                cd1_eq_right_side= transpose_qdot_vec_k*(q_vec/(q_vec_norm*(q_vec_norm-small_delta)));
                %%%%SUBJECT TO (2)   
                cd2_eq_right_side= transpose_qdot_vec_l*(-q_vec/(q_vec_norm*(q_vec_norm-small_delta)));

                c(ii)=(-cd1_eq_left_side+cd1_eq_right_side);
                c(ii+1)=(-cd2_eq_left_side+cd2_eq_right_side);
                ii= ii+2;
            end
           end
        end
    end
end


ceq=[];