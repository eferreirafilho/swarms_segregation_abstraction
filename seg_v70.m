%Simulation of the segregation of a swarm with n robots
%edsonbffilho@gmail.com
%22-Mar-2019 - Groups now can be unbalanced
%25-Mar-2019 - Robots added and withdrawn

clear all

%M = number of groups
%n_abs=3;
for irng=10:10
tic
%rng(1)
rng(irng)
irng

close all

%Send? [0] no [1] yes
send_robotarium=1;

%number of robots per group
%n_robots=3;
%N=n_robots*n_abs;
%M=[2,2,2,2];%Number of robots per group
M=[3,3,3,2,2];%Number of robots per group
n_abs=size(M,2);
N=sum(M);

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%Plot? [1] Yes [0] No
flag_plot=1;
plot_interval=1;%200

%Radius of each robot
%radius=0.03;
radius=0.5*r.robot_diameter;

%% Experiment constants

% Select the number of iterations for the experiment.  This value is
% arbitrary
%iterations = 2300;
iterations = 3000;

%Collision avoidance [1]on [0]off
enable_collision_avoidance=1;

%Energy Dissipation [1] on [0] off
energy_dissipation=0;

%Groups can change size? [1] on [0] off
change_groups=0;

%Delta to enter the collision avoidance
%delta_collision=2*r.robot_diameter; <-best
delta_collision=2*r.robot_diameter;

%Distribution of robots
dist_var=max(M)*n_abs^2;

%% Initialize Variables
UUx(1:max(M),1:n_abs)=0;
UUy(1:max(M),1:n_abs)=0;
vvx(1:max(M),1:n_abs)=0;
vvy(1:max(M),1:n_abs)=0;
UUx(1:max(M),1:n_abs)=0;
UUy(1:max(M),1:n_abs)=0;
rx(1:n_abs)=0;
ry(1:n_abs)=0;
vx(1:n_abs)=0;
vy(1:n_abs)=0;
Ux(1:n_abs)=0;
Uy(1:n_abs)=0;
Vx(1:n_abs)=0;
Vy(1:n_abs)=0;
Fx(1:n_abs)=0;
Fy(1:n_abs)=0;
cont_coli=0;
sigma_error_hist(1:n_abs,iterations)=0;
true_sigma_error_hist(1:n_abs,iterations)=0;
true_sigma_hist(1:n_abs,iterations)=0;
rrx(1:max(M),1:n_abs)=0;
rry(1:max(M),1:n_abs)=0;
rrx_pure(1:max(M),1:n_abs)=0;
rry_pure(1:max(M),1:n_abs)=0;
vvx_pure(1:max(M),1:n_abs)=0;
vvy_pure(1:max(M),1:n_abs)=0;
sigma_controller_x(1:max(M),1:n_abs)=0;
sigma_controller_y(1:max(M),1:n_abs)=0;
flag_collision(1:iterations,1:n_abs)=0;
norm_r(1:N,1:N)=0;
ca_norm_groups(1:N,1:N)=0;
rrx_ca_hist(1:N,1:iterations)=0;
rry_ca_hist(1:N,1:iterations)=0;
rrx_ca_pure_hist(1:N,1:iterations)=0;
rry_ca_pure_hist(1:N,1:iterations)=0;
vvx_ca_hist(1:N,1:iterations)=0;
vvy_ca_hist(1:N,1:iterations)=0;
vvx_ca_pure_hist(1:N,1:iterations)=0;
vvy_ca_pure_hist(1:N,1:iterations)=0;
UUx_ca_hist(1:N,1:iterations)=0;
UUy_ca_hist(1:N,1:iterations)=0;
adj_mat(1:n_abs,1:n_abs)=0;
norm_groups(1:n_abs,1:n_abs)=0;
h1=gobjects(n_abs,max(M));
h2=gobjects(n_abs,1);
h3=gobjects(n_abs,max(M));
sigma_des(1:n_abs)=0;
sigma1(1:n_abs)=0;
sigma3(1:n_abs)=0;
sigma_dot(1:n_abs)=0;
d_phi_null_x{1,n_abs}=[];
d_phi_null_y{1,n_abs}=[];
d_phi_null_v{1,n_abs}=[];
d_phi_null{1,n_abs}=[];

%Change groups size
if change_groups==1
    sigma_hist(1:n_abs+1,1:iterations)=0;
end

%% Potential function parameters
%sigma_des=0.04;

%d=sqrt(6.5*n_robots*sigma_des); %Mean of initial Sigma
d=0.87;%1.02
for j=1:n_abs
    sigma_des(j)=-0.004+(d^2)/(4*M(j));
end

eps_norm_sigma=0.5; %Epsilon of the sigma norm
fator=1.71; 
r_potfn=fator*d;        %r<sqrt(3)*d (Not Used)
d_alpha=(1/eps_norm_sigma)*((sqrt(1+eps_norm_sigma*(d^2)))-1);
r_alpha=fator*d_alpha;        %r<2*d
a=1;
b=1;
h=0.3;  %h>0.5 -> unstable
c=(a-b)/(sqrt(4*a*b));
pot_param=[eps_norm_sigma,r_potfn,d_alpha,r_alpha,a,b,h,c];


%% Sigma Controller Parameters

%Sigma Gains
kp=0.1; %3%P
kd=0.005; %D  

%% %Random color
if n_abs>10
    rand_color = rand(n_abs,3);
else
    rand_color(1:10,1:3)=[1 0 0;0 1 0;0 0 1;0 1 1;0 0 0;0.3 0.5 0.7;0.3 0.7 0;1 1 0;0.9 0.9 0.9;0.3 0.3 0.3];
end

%% Which Group Function
aux_ca_wg=(1:N);
cont=1;
for k=1:n_abs
    %wg(n_robots*k-(n_robots-1):n_robots*k)=aux_ca_wg(:,k);
    wg(cont:(cont+M(k)-1))=aux_ca_wg(:,k);
    cont=cont+M(k);
end
options = optimoptions('fmincon','Algorithm','active-set','Display','off','TolFun',1e-4,'TolCon',1e-4);

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();

%safety_radius = 1.4*r.robot_diameter;
%si_barrier_cert = create_si_barrier_certificate('SafetyRadius', safety_radius);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

% Initialize dxu
dxu = zeros(2, N);

%The time delay is approximately 0.033 seconds
i_step=0.033;

%Distributed robots randonly but more centralized in the enviroment
rand_init_pos(:,1)=-1.4 + (1.4+1.4).*rand(N,1);%X
rand_init_pos(:,2)=-0.8 + (0.8+0.8).*rand(N,1);%Y

% Initialize all plots.
hold on;

% Set handlers.
color = hsv(n_abs);
handler_robots = zeros(1, n_abs);
list_markers=['p','d','s','h','p'];
cont=1;
if send_robotarium==1
    marker_size=70;
else
    marker_size=7;
end
for i = 1:n_abs
    %Initialize Robot plot
    handler_robots(i) = plot(nan, list_markers(i));
    set(handler_robots(i),'Color',rand_color(i,:), 'MarkerSize', marker_size,'MarkerFaceColor',rand_color(i,:));
    set(handler_robots(i),'XData',dxi(1,cont:(cont+M(i)-1)),'YData',dxi(2,cont:(cont+M(i)-1)));
    
    %Initialize Abstractions Plot
    handler_abs(i) =  plot(nan, '.');
    set(handler_abs(i),'Color',rand_color(i,:), 'MarkerSize', 5);
    theta = 0 : 0.01 : 2*pi;
    xA = 0;
    yA = 0;
    set(handler_abs(i),'XData',xA,'YData',yA);
    
    %Initialize Noise plot
    handler_noise(i) = plot(nan, list_markers(i));
    set(handler_noise(i),'Color',rand_color(i,:), 'MarkerSize', 15,'MarkerFaceColor',rand_color(i,:));
    set(handler_noise(i),'XData',dxi(1,cont:(cont+M(i)-1)),'YData',dxi(1,cont:(cont+M(i)-1)));
    cont=cont+M(i);
end

% for t=1:500
%     %% Retrieve the most recent poses from the Robotarium.  The time delay is
%     % approximately 0.033 seconds
%     x = r.get_poses();
%     
%     % Convert to SI states
%     xi = uni_to_si_states(x);
%     
%     %% Applying controllers to the robots
%     dxi(1,:)=0.02*(rand_init_pos(:,1)'-xi(1,:));
%     dxi(2,:)=0.02*(rand_init_pos(:,2)'-xi(2,:));
%     
%    %% Avoid errors
%     
%     % To avoid errors, we need to threshold dxi
%     norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
%     threshold = 3/4*r.max_linear_velocity;
%     to_thresh = norms > threshold;
%     dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
%     
%     %% Map SI to Uni dynamics and utilize barrier certificates
%     
%     % Transform the single-integrator to unicycle dynamics using the the
%     % transformation we created earlier
%     dxu = si_to_uni_dyn(dxi, x);
%     
%     dxu = uni_barrier_cert_boundary(dxu, x);
%     
%     %% Send velocities to agents
%     
%     % Set velocities of agents 1,...,N
%     r.set_velocities(1:N, dxu);
%     
%     % Send the previously set velocities to the agents.  This function must be called!    
%     r.step();
% end

robots_ok=1;

%Iterate for the previously specified number of iterations
for t = 1:iterations
    if robots_ok==1
    
    %% Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xi = uni_to_si_states(x);   
    
    %Change groups
    if change_groups==1
        if t>1200
            M=[2,2,3,3];%Number of robots per group
            n_abs=size(M,2);
            N=sum(M);
            for j=1:n_abs
                sigma_des(j)=-0.004+(d^2)/(4*M(j));
            end
            %% Which Group Function
            aux_ca_wg=(1:N);
            cont=1;
            for k=1:n_abs
                %wg(n_robots*k-(n_robots-1):n_robots*k)=aux_ca_wg(:,k);
                wg(cont:(cont+M(k)-1))=aux_ca_wg(:,k);
                cont=cont+M(k);
            end
        end
        if t>2100
            M=[4,3,3];%Number of robots per group
            n_abs=size(M,2);
            N=sum(M);
            for j=1:n_abs
                sigma_des(j)=-0.004+(d^2)/(4*M(j));
            end
            %% Which Group Function
            aux_ca_wg=(1:N);
            cont=1;
            for k=1:n_abs
                wg(cont:(cont+M(k)-1))=aux_ca_wg(:,k);
                cont=cont+M(k);
            end
        end
    end
    
    %Noise
    noise=randn(N,4);
    noise_percent=10;
    noise(:,1)=(noise_percent/100)*3.3*noise(:,1);
    noise(:,2)=(noise_percent/100)*2.2*noise(:,2);
    noise(:,3:4)=0*noise(:,1:2);
    
    %Get velocities (New way)
    vvx_ca(1:N,1)=(dxu(1,:).*cos(x(3,:)));
    vvy_ca(1:N,1)=(dxu(1,:).*sin(x(3,:)));
    
    %Get Positions
    rrx_ca(1:N,1)=x(1,:);
    rry_ca(1:N,1)=x(2,:);
    
    %Verify if robots are outside area (only in simulation)
    if send_robotarium==0
        for i=1:N
            if rrx_ca(i,1)>1.65 || rrx_ca(i,1)<-1.65 || rry_ca(i,1)>1.05 || rry_ca(i,1)<-1.05
                robots_ok=0;
            end
        end
    end
    
    %Save pure positions and velocities
    rrx_ca_pure_hist(:,t)=x(1,:);
    rry_ca_pure_hist(:,t)=x(2,:);
    vvx_ca_pure_hist(:,t)=vvx_ca;
    vvy_ca_pure_hist(:,t)=vvy_ca;
    
    %% Algorithm   
    %%  Potential Function Calculation Start
    
    %Real Abstractions Mean    
    %Transform Variables
    cont=1;
    for i=1:n_abs
        rrx(1:M(i),i)=rrx_ca(cont:(cont+M(i)-1)) + noise(cont:(cont+M(i)-1),1);%Noise Added
        rry(1:M(i),i)=rry_ca(cont:(cont+M(i)-1)) + noise(cont:(cont+M(i)-1),2);%Noise Added
        vvx(1:M(i),i)=vvx_ca(cont:(cont+M(i)-1)) + noise(cont:(cont+M(i)-1),3);%Noise Added
        vvy(1:M(i),i)=vvy_ca(cont:(cont+M(i)-1)) + noise(cont:(cont+M(i)-1),4);%Noise Added
        %States without Noise
        rrx_pure(1:M(i),i)=rrx_ca(cont:(cont+M(i)-1));
        rry_pure(1:M(i),i)=rry_ca(cont:(cont+M(i)-1));
        vvx_pure(1:M(i),i)=vvx_ca(cont:(cont+M(i)-1));
        vvy_pure(1:M(i),i)=vvy_ca(cont:(cont+M(i)-1));
        
        %Calculate mean
        rx(i)=mean(rrx(1:M(i),i));
        ry(i)=mean(rry(1:M(i),i));
        %Abstractions Mean velocity
        vx(i)=mean(vvx(1:M(i),i));
        vy(i)=mean(vvy(1:M(i),i));
        cont=cont+M(i);
    end

    %Potencial forces
    for j=1:n_abs
        mu_est_x=rx(j);
        mu_est_y=ry(j);
        group_number=j;           
        [potfnx,potfny]=pot_function(mu_est_x,mu_est_y,rx,ry,group_number,eps_norm_sigma,r_potfn,d_alpha,r_alpha,a,b,h,c);
        %Olfati-saber forces for each group
        Ux(j)=sum(potfnx);
        Uy(j)=sum(potfny);
    end
    
    for j=1:n_abs
            %Uses Velocities Mean positions
            mu_est_x=rx(j);
            mu_est_y=ry(j);
            mu_est_x_dot=vx(j);
            mu_est_y_dot=vy(j);
            group_number=j;           
            [Vx_damping,Vy_damping]=pot_function_damping(mu_est_x,mu_est_y,mu_est_x_dot,mu_est_y_dot,rx,ry,vx,vy,r_alpha,h,eps_norm_sigma,group_number);
            %Olfati-saber damping forces that each robot
            Vx(j)=sum(Vx_damping(j,:));
            Vy(j)=sum(Vy_damping(j,:));
    end
               
    %Potencial Forces + Damping Term
    Fx = Ux + Vx;
    Fy = Uy + Vy;
    
    %% Sigma Controller  
    %Real Sigma
    for i=1:n_abs
        sigma1(i)=var(rrx(1:M(i),i));
        sigma3(i)=var(rry(1:M(i),i));
        %Calculated without noise
        true_sigma1(i)=var(rrx_pure(1:M(i),i));
        true_sigma3(i)=var(rry_pure(1:M(i),i));
    end
    sigma=sigma1+sigma3;    
    %Calculated without noise
    true_sigma=true_sigma1+true_sigma3;   
    
    %Real Sigma Dot of each abstraction
    sigma_dot1(1:n_abs)=0;
    sigma_dot3(1:n_abs)=0;
    sigma_line(1:n_abs)=0;
   
    for j=1:n_abs
        for i=1:M(j)
            sigma_dot1(j)=sigma_dot1(j) + ((rrx(i,j) - rx(j)))*vvx(i,j);
            sigma_dot3(j)=sigma_dot3(j) + ((rry(i,j) - ry(j)))*vvy(i,j);
            %Sigma' of each abstraction
            sigma_line(j)=sigma_line(j) + (1/M(j))*((vvx(i,j) - vx(j))^2) + (1/M(j))*((vvy(i,j) - vy(j))^2);
        end
    end 
    for j=1:n_abs
        sigma_dot(j)=(2/M(j))*(sigma_dot1(j)+sigma_dot3(j));
    end
    
    sigma_error_hist(1:n_abs,t)=abs(sigma-sigma_des);
    true_sigma_error_hist(1:n_abs,t)=abs(true_sigma-sigma_des);

       
    for j=1:n_abs
        for i=1:M(j)
            sigma_controller_x(i,j)=((rrx(i,j)-rx(j))/(sigma(j)))*(-2*sigma_line(j)+kp*(sigma_des(j)-sigma(j)) - kd*(sigma_dot(j)));
            sigma_controller_y(i,j)=((rry(i,j)-ry(j))/(sigma(j)))*(-2*sigma_line(j)+kp*(sigma_des(j)-sigma(j)) - kd*(sigma_dot(j)));             
        end
    end 
    
    %% Individual Control Laws      
     for j=1:n_abs
        for i=1:M(j)
            UUx(i,j)=Fx(j) + sigma_controller_x(i,j);
            UUy(i,j)=Fy(j) + sigma_controller_y(i,j);   
        end
     end
     
    %% Collision Avoidance
    flag_collision(t,1:n_abs)=0;
    if enable_collision_avoidance==1
         
    % Distance between agent i with agent j
    for i=1:N
    for j=1:N
        norm_r(i,j)=sqrt((rrx_ca(i) - rrx_ca(j))*(rrx_ca(i) - rrx_ca(j)) + (rry_ca(i) - rry_ca(j))*(rry_ca(i) - rry_ca(j)));
          if(norm_r(i,j)<2*radius && i~=j)
              %disp('COLLISION!')
              flag_collision(t,i)=1;
              cont_coli=cont_coli+1;
          end
          if i==j
              norm_r(i,j)=NaN;
          end
    end
    end
         
    % Null Space Calculations
    dphi=0;
    for k=1:n_abs
        dphi_aux=0;
        dphi_aux(1:2*M(k),1:3)=0;
        j=1;       
        for i=1:2:2*M(k)
            dphi_aux(i,1)=1;
            dphi_aux(i,3)=(2/M(k))*(rrx(j,k) - rx(k));
            j=j+1;
        end
        j=1;
        for i=2:2:2*M(k)
            dphi_aux(i,2)=1;
            dphi_aux(i,3)=(2/M(k))*(rry(j,k) - ry(k));
            j=j+1;
        end       
        %dphi_aux
        dphi=dphi_aux';
        d_phi_null{k}=eye(M(k)*2) - transpose(dphi)*(inv(dphi*transpose(dphi)))*dphi;
    end
     
    %index_r -> robot is in imminent collision?
    %ca_norm -> which robots are in imminent collision
    [index_r,ca_norm,norm_r]=robots_in_imminent_collision(rrx_ca,rry_ca,radius,delta_collision,M);
    
    %% NEW COLLISION AVOIDANCE -> INDIVIDUAL   
        %Collision Graph: Calculate Sigma_p (as in Journal)
        %ca_norm with the groups robots belongs  
        for i=1:N
            for j=1:N  
                if ca_norm(i,j)~=0
                    ca_norm_groups(i,j)=wg(i);
                else
                    ca_norm_groups(i,j)=0;
                end
            end
        end

      G_collision_adj(1:n_abs,1:n_abs)=0;
      for i=1:N             
          %Which Group I belong
          nn_auxi=wg(i);
            %Which groups Im colliding
            for j=1:N
                if ca_norm_groups(j,i)~=0
                    nn_auxj=wg(j);
                    G_collision_adj(nn_auxi,nn_auxj)=1;
                end
            end
     end
    
   G_collision_adj;
   G_collision = graph(G_collision_adj);
   bins = conncomp(G_collision);
   binnodes = accumarray(bins', 1:numel(bins), [], @(v) {sort(v')});
   
   %Number of connected collision components
   p_sigma=0;
   p_sigma_idx=0;
   cont_binidx=1;
for binidx = 1:numel(binnodes)
    if length(binnodes{binidx})~=1
    %fprintf('All these nodes are connected:%s\n', sprintf(' %d', binnodes{binidx}));
    p_sigma=p_sigma+1;
    p_sigma_idx(cont_binidx)=binidx;
    cont_binidx=cont_binidx+1;
    end
end

%% %Two Situations:
% %1 - Collision Only inside group 
% %2 - Collision between different groups
% 
%% %1 - Collision only in one group
own_group_ca=0;
for i=1:length(binnodes)   
     %Grupo alone in the collision graph
     if length(binnodes{i})==1
         %Collisions inside same group?
         if G_collision_adj(binnodes{i},binnodes{i})==0
             %disp('Do not have collision inside group')
             %disp(i)
         else
             %disp('Collisions inside group')
             %disp(i)
             own_group_ca(i)=binnodes{i};
         end                       
     end              
end
 
%Run Minimization For each group that have colissions inside own group
%For each group in this situation

for j=1:n_abs
    d_phi_null_x{j}=zeros(M(j),1);
    d_phi_null_y{j}=zeros(M(j),1);
    d_phi_null_x_own{j}=zeros(M(j),1);
    d_phi_null_y_own{j}=zeros(M(j),1);
    a_new{j}=zeros(1,2*M(j));
    d_phi_null_v{j}=zeros(1,2*M(j));
end
for k=1:length(own_group_ca)    
     %Group Index
     if own_group_ca(k)~=0
         inside_ca=1;%Has entered CA this step?
         group_idx=own_group_ca(k);
         %Which robots in collision?
         u_hat_own=0;
         u_hat_own(1:2*M(group_idx),1)=0;
         [u_hat_own,FVAL,EXITFLAG,OUTPUT]=fmincon(@(u_hat_own)new_fun_ca(u_hat_own),u_hat_own,[],[],[],[],[],[],@(u_hat_own)new_con_ca_own(u_hat_own,ca_norm,d_phi_null,UUx,UUy,radius,rrx,rry,vvx,vvy,group_idx,delta_collision,M),options);         
         
         %NULL SPACE CALCULATIONS START
         d_phi_null_v_own(1:2*M(group_idx))=d_phi_null{group_idx}*u_hat_own;
         %Transforming null space to x and y
         cont_c=1;
         for ii=1:2:2*M(group_idx)
              d_phi_null_x_own{group_idx}(cont_c)=d_phi_null_v_own(ii);
              cont_c=cont_c+1;
         end
         cont_c=1;
         for ii=2:2:2*M(group_idx)
              d_phi_null_y_own{group_idx}(cont_c)=d_phi_null_v_own(ii);
              cont_c=cont_c+1;
         end
         %NULL SPACE CALCULATIONS END 
     end
     d_phi_null_x{j}=d_phi_null_x_own{j};
     d_phi_null_y{j}=d_phi_null_y_own{j};
end

%% %2 - Collision between different groups
%Run Minimization For each group that have colissions with other groups
%Number of connected components of graph (Excluding only inside own group)
    inside_ca=0;
    for i=1:p_sigma
        inside_ca=1;%Has entered CA this step?
        %Which groups are in collision
        groups_idx=binnodes{p_sigma_idx(i)};
        %u_hat has is stacked with all groups
        u_hat(1:2*N)=0;
        [u_hat,FVAL,EXITFLAG,OUTPUT]=fmincon(@(u_hat)new_fun_ca(u_hat),u_hat,[],[],[],[],[],[],@(u_hat)new_con_ca_groups(u_hat,ca_norm,d_phi_null,UUx,UUy,radius,rrx,rry,vvx,vvy,groups_idx,delta_collision,N,M,wg),options);        
        cont=1;
        %Transform variables
        for k=1:n_abs
            a_new{k}=u_hat(cont:(cont+2*M(k)-1));
            cont=cont+2*M(k);
        end
        %NULL SPACE CALCULATIONS START
        for k=1:n_abs
            d_phi_null_v{k}=d_phi_null{k}*a_new{k}';
        end      
        %Transforming null space to x and y
        for k=1:n_abs
            cont_c=1;
            for ii=1:2:2*M(k)
                d_phi_null_x{k}(cont_c)=d_phi_null_v{k}(ii);
                cont_c=cont_c+1;
            end
                cont_c=1;
            for ii=2:2:2*M(k)
                d_phi_null_y{k}(cont_c)=d_phi_null_v{k}(ii);
                cont_c=cont_c+1;
            end
        end
    %NULL SPACE CALCULATIONS END   
    end

    end

    %% Individual Control Laws with Collision Avoidance
    for j=1:n_abs
        UUx(1:M(j),j)=UUx(1:M(j),j)+d_phi_null_x{j};
        UUy(1:M(j),j)=UUy(1:M(j),j)+d_phi_null_y{j};
    end
    
    %Transforming variables
    cont=1;
    for i=1:n_abs
        rrx_ca(cont:(cont+M(i)-1),1)=rrx(1:M(i),i);
        rry_ca(cont:(cont+M(i)-1),1)=rry(1:M(i),i);
        UUx_ca(cont:(cont+M(i)-1),1)=UUx(1:M(i),i);
        UUy_ca(cont:(cont+M(i)-1),1)=UUy(1:M(i),i);
        cont=cont+M(i);
    end
        
    %Adj_mat for abstractions (Used to Draw)
%     adj_mat(n_abs,n_abs)=0;
%     for i=1:n_abs
%     for j=1:n_abs
%         norm_groups(i,j)=sqrt((rx(i) - rx(j))*(rx(i) - rx(j)) + (ry(i) - ry(j))*(ry(i) - ry(j)));
%         if norm_groups(i,j)<r_potfn && i~=j
%             adj_mat(i,j)=1;
%         else
%             adj_mat(i,j)=0;
%         end
%     end
%     end
%         %Draw connections between groups
%         draw_graph(rx',ry',adj_mat,[0.2,0.2,0.2],0.2)

    %% Plot
    if flag_plot==1
        for i=1:n_abs
            %h2(i)=circle(rx(i),ry(i),sqrt(M(i)*sigma(i)),rand_color(i,1:3));
            %Draw With pure values
            %h2(i)=circle(mean(rrx_pure(1:M(i),i)),mean(rry_pure(1:M(i),i)),sqrt(M(i)*(var(rrx_pure(1:M(i),i))+var(rry_pure(1:M(i),i)))),rand_color(i,1:3));
        end
        
        cont=1;
        for i=1:n_abs

                %Update Robots Plot
                start = floor((i - 1) * N / n_abs) + 1;
                stop  = floor(i * N / n_abs);
                set(handler_robots(i),'XData',x(1,cont:(cont+M(i)-1)),'YData',x(2,cont:(cont+M(i)-1)));
                
                %Update Abstractions Plot
                xA = sqrt(M(i)*true_sigma(i)) * cos(theta) + mean(rrx_pure(1:M(i),i));
                yA = sqrt(M(i)*true_sigma(i)) * sin(theta) + mean(rry_pure(1:M(i),i));
                set(handler_abs(i),'XData',xA,'YData',yA);    
                
                set(handler_noise(i),'XData',rrx_ca(cont:(cont+M(i)-1),1),'YData',rry_ca(cont:(cont+M(i)-1),1));
                cont=cont+M(i);
        end

        if mod(t/50,1) == 0
            delete(h1)
            h1=text(-1.55,-0.9,sprintf('Iteration: %d', t));
        end
        
        drawnow
        
%         delete(h2)
%         delete(h3)
    end
    
    %% Save hist
    rrx_ca_hist(:,t)=rrx_ca;
    rry_ca_hist(:,t)=rry_ca;
    vvx_ca_hist(:,t)=vvx_ca;
    vvy_ca_hist(:,t)=vvy_ca;
    UUx_ca_hist(:,t)=UUx_ca;
    UUy_ca_hist(:,t)=UUy_ca;
    sigma_hist(1:n_abs,t)=sigma(1:n_abs);
    true_sigma_hist(1:n_abs,t)=true_sigma(1:n_abs);
    noise_hist(:,:,t)=noise(:,:);
        
    %% Transform the double-integrator controller to single-integrator dynamics
    UUx_ca=vvx_ca+i_step*UUx_ca;
    UUy_ca=vvy_ca+i_step*UUy_ca;
    
    %% If abstractions are far, stop robots
%     sum_groups=sum(adj_mat);
%     for j=1:n_abs
%         if sum_groups(j)==0
%             UUx_ca(n_robots*j-(n_robots-1):n_robots*j)=0;
%             UUy_ca(n_robots*j-(n_robots-1):n_robots*j)=0;
%             disp('stop group')
%             disp(j)
%         end
%     end
%     

    %% Applying controllers to the robots
    dxi(1,:)=UUx_ca;
    dxi(2,:)=UUy_ca;
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Map SI to Uni dynamics and utilize barrier certificates
    
    % Transform the single-integrator to unicycle dynamics using the the
    % transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    dxu = uni_barrier_cert_boundary(dxu, x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
    end
    
end
toc

%Print sigma error Debug
%sigma_error_hist(:,t)

%Save data
save('alldata.mat')

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
end

