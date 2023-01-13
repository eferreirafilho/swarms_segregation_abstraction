function [c,ceq]= new_con_ca_own(u_hat_own,ca_norm,d_phi_null,UUx,UUy,radius,rrx,rry,vvx,vvy,group_idx,delta_collision,M)

%n_abs=size(rrx,2);%Only one group being treated here!
n_robots=M(group_idx);

%Transforming Variables of group being treated
rrx_ca=rrx(1:M(group_idx),group_idx);
rry_ca=rry(1:M(group_idx),group_idx);
vvx_ca=vvx(1:M(group_idx),group_idx);
vvy_ca=vvy(1:M(group_idx),group_idx);
UUx_ca=UUx(1:M(group_idx),group_idx);
UUy_ca=UUy(1:M(group_idx),group_idx);
%u_hat has only one group


%ca_norm only for the group being treated
%ca_norm_aux=ca_norm(n_robots*group_idx-(n_robots-1):n_robots*group_idx,n_robots*group_idx-(n_robots-1):n_robots*group_idx);      

d_phi_null_own=d_phi_null{group_idx};%(group_idx*n_robots*2-n_robots*2+1:group_idx*n_robots*2,group_idx*n_robots*2-n_robots*2+1:group_idx*n_robots*2);

%d_phi_null_v_own -> null space control inputs X and Y of one group

d_phi_null_v_own=d_phi_null_own*u_hat_own;

d_phi_null_x_con(1:2*n_robots)=0;
d_phi_null_y_con(1:2*n_robots)=0;
%Transforming null space to x and y
cont_c=1;
for i=1:2:2*n_robots
    d_phi_null_x_con(cont_c)=d_phi_null_v_own(i);
    cont_c=cont_c+1;
end
cont_c=1;
for i=2:2:2*n_robots
    d_phi_null_y_con(cont_c)=d_phi_null_v_own(i);
    cont_c=cont_c+1;
end


%UUx_new_ca=0;
%UUy_new_ca=0;
%Adding the CA inputs to old inputs
UUx_new_ca=UUx_ca+d_phi_null_x_con;
UUy_new_ca=UUy_ca+d_phi_null_y_con;
 

u_hat_old(1:n_robots,1)=0;
%% Add collisions for each pair of robots in imminent collision
ii=1; %Count Constraints
c=[];  %if there is not a pair
for i=1:n_robots
    for j=1:n_robots
        if (ca_norm(i,j)==1 && i~=j)
            %Adding one new constraint
            qdot_vec_k=[(vvx_ca(i));(vvy_ca(i))];
            qdot_vec_l=[(vvx_ca(j));(vvy_ca(j))];
            q_vec=[(rrx_ca(i) - rrx_ca(j));(rry_ca(i) - rry_ca(j))];
            %Now, the new input (UUx_new) must satisfie the constraints
            q_vec_norm=sqrt((rrx_ca(i) - rrx_ca(j))^2 + (rry_ca(i) - rry_ca(j))^2);
            %disp(i)
            %disp(j)
            for iii=1:2*n_robots
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
            %for small_delta=radius-0.5*radius:0.2:4*radius %Layers define multiple constraints
            for small_delta=radius
            %for small_delta=radius-0.5*radius:0.2:4*radius
            %for small_delta=radius-0.5*radius:0.2:delta_collision

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

ceq=[];