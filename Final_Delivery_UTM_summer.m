clc
clear;
close all;


%Set RUN_SIM to true to run the flight simulation
RUN_SIM = true;
CRASH_PLOT = false;
MOVIE = true;

%% initialize everything
mySim = initialize_simulation();

drone1 = initialize_drone();
drone2 = initialize_drone();

drone1.data.x = zeros(8,mySim.idx_end);
drone1.data.u = zeros(4,mySim.idx_end);
drone2.data.x = zeros(8,mySim.idx_end);
drone2.data.u = zeros(4,mySim.idx_end);

dist_12 = zeros(1,mySim.idx_end); %Distance between 1 and 2
coll_12 = zeros(1,mySim.idx_end); %Collision between 1 and 2

%% Load up Background Map
I = imread('city_map2.jpg');
J = flipud(I);
figure;
imagesc(J); hold on;
set(gca,'YDir','normal');
grid on;
axis equal;
xlim([1 1463])
ylim([1 920])   


%%


Obstacles = Create_Obstacle_Map();
plot_Obs(Obstacles);
Obs = build_total_points(Obstacles);

Drone_Targets = Create_Drone_Targets();
%Fill out the list:
%Drone_Target[0]: Launch Point
%Drone_Target[1]: Launch Point
%Drone_Target[2]: Delivery
%Drone_Target[3]: _________
%Drone_Target[4]: _________
%Drone_Target[5]: _________
%Drone_Target[6]: _________
%Drone_Target[7]: _________
%Drone_Target[8]: _________
plot_DroneTargets(Drone_Targets);

plot_Voronoi_Map(Obs); 

[v,G] = Generate_Voronoi(Obs, 0.8);
disp('Finished Voronoi Map');


%%

drone1.s(1:2) = Drone_Targets(1).points;
drone1.s(5) = deg2rad(0);


drone2.s(1:2) = Drone_Targets(2).points;
drone2.s(5) = deg2rad(270);

Flight_Plan_1 = initialize_flight_plan();
Flight_Plan_2 = initialize_flight_plan();



%% Generate Waypoints

[waypoints_1_to_3,score13] = Generate_Best_Path_Around_Obstacles(G,v,Drone_Targets(1).points,Drone_Targets(3).points);
[waypoints_3_to_4,score34] = Generate_Best_Path_Around_Obstacles(G,v,Drone_Targets(3).points,Drone_Targets(4).points);
[waypoints_2_to_5,score25] = Generate_Best_Path_Around_Obstacles(G,v,Drone_Targets(2).points,Drone_Targets(5).points);
[waypoints_3_to_1,score31] = Generate_Best_Path_Around_Obstacles(G,v,Drone_Targets(3).points,Drone_Targets(1).points);



%Compare different score values. What could you use this information for?


disp('Waypoints Generated');

%%

if RUN_SIM

mySim.sim_flag = true;
mySim.sim_flag1 = true;
mySim.sim_flag2 = true;

i = 1;
while mySim.sim_flag == true
    if (mod(i,100)==0)
        clc;
        str = "Progress: " + i + " frame";
        disp(str);
    end
    %Move aircraft information from the x state vector to more readable
    %variables.  
    pn1 = drone1.s(1);
    pe1 = drone1.s(2);
    h1 = drone1.s(3);
    
    pn2 = drone2.s(1);
    pe2 = drone2.s(2);
    h2 = drone2.s(3);
    
    
    %% State Machine 1
    % State 0 - Launch %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (Flight_Plan_1.State == 0)
        %Output
        drone1.u.Height = 18; 
        drone1.u.AirSpeed = 5;
        drone1.u.Xi_c = drone1.s(5);
        
        %Trigger
        if (h1>17) %If __________
            Flight_Plan_1.State = 1;
            mySim.start_loop_time = mySim.t; %Start timer
            Flight_Plan_1 = update_waypoints(Flight_Plan_1,waypoints_1_to_3,drone1);
        end
        
    % State 1 - Follow Waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_1.State == 1)
        % Calculations
        
        [flag,r,q,c,rho,lambda,Flight_Plan_1] = followWaypoints(Flight_Plan_1,drone1);   
        
        %Output
        drone1.u.Height = 25; 
        drone1.u.AirSpeed = 5;
        drone1.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone1);
        
        %Trigger
        if Flight_Plan_1.waypoint_target>Flight_Plan_1.n_waypoints
            
            mySim.start_loop_time = mySim.t; %Restart timer
            Flight_Plan_1.State = 2; 
            Flight_Plan_1 = update_circle(Flight_Plan_1,Drone_Targets(3).points);
        end
    
    %State 2 - Do circles
    elseif (Flight_Plan_1.State == 2)
        flag = 2;
        c = Flight_Plan_1.circle_center;
        rho = Flight_Plan_1.loiter_radius;
        lambda = 1;
        r = 0;
        q = 0;
        
        %Output
        drone1.u.Height = change_altitude(-20*mySim.dt,10,drone1);
        drone1.u.AirSpeed = 5;
        drone1.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone1);
        
        Flight_Plan_1 = count_circles(drone1,Flight_Plan_1,c);
        
        if (Flight_Plan_1.loop_count > 2)
            Flight_Plan_1.State = 3;
            Flight_Plan_1 = update_waypoints(Flight_Plan_1,waypoints_3_to_1,drone1);
        end
        
    %State 3 - Do waypoints:     
    elseif (Flight_Plan_1.State == 3)
        % Calculations
        
        [flag,r,q,c,rho,lambda,Flight_Plan_1] = followWaypoints(Flight_Plan_1,drone1);   
        
        %Output
        drone1.u.Height = 25; 
        drone1.u.AirSpeed = 5;
        drone1.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone1);
        
        %Trigger
        if Flight_Plan_1.waypoint_target>Flight_Plan_1.n_waypoints
            
            mySim.start_loop_time = mySim.t; %Restart timer
            Flight_Plan_1.State = 4; 
            Flight_Plan_1 = update_circle(Flight_Plan_1,Drone_Targets(1).points);
        end
        
    %State 4 - do circles until landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_1.State ==4)
        flag = 2;
        c = Flight_Plan_1.circle_center;
        rho = Flight_Plan_1.loiter_radius;
        lambda = 1;
        r = 0;
        q = 0;
        
        %Output
        drone1.u.Height = change_altitude(-20*mySim.dt,10,drone1);
        drone1.u.AirSpeed = 5;
        drone1.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone1);
        
        
        Flight_Plan_1 = count_circles(drone1,Flight_Plan_1,c);
        %Trigger
        if (Flight_Plan_1.loop_count > 2) 
            Flight_Plan_1.State = Flight_Plan_1.LANDING; 
            mySim.start_loop_time1 = mySim.t; %Restart timer
        end
        
 
    %State LANDING - Come in for a landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_1.State == Flight_Plan_1.LANDING)
        %calculations
        mySim.loop_time1 = mySim.t - mySim.start_loop_time1; %Check timer
        
        %Output
        drone1.u.Height = change_altitude(-20*mySim.dt,0,drone1);
        drone1.u.AirSpeed = change_speed(-10*mySim.dt,1,drone1);
        drone1.u.Xi_c = deg2rad(0);
        
        %Trigger
        if (h1<1) %If __________
            Flight_Plan_1.State = Flight_Plan_1.TOUCHDOWN;
            mySim.start_loop_time1 = mySim.t; %Restart timer
        end
        
    
    % State TOUCHDOWN - Wait 10 seconds after landing %%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_1.State == Flight_Plan_1.TOUCHDOWN)
        %calculations
        mySim.loop_time1 = mySim.t - mySim.start_loop_time1; %Check timer
        
        %Output
        drone1.u.Height = 0;
        drone1.u.AirSpeed = 0;
        drone1.u.Xi_c = deg2rad(0);
             
        %Trigger
        if (mySim.loop_time1 > 10)
            mySim.sim_flag = false;
        end
    end
    
    %% State Machine 2
    % State 0 - Launch %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (Flight_Plan_2.State == 0)
        %Output
        drone2.u.Height = 18; 
        drone2.u.AirSpeed = 5;
        drone2.u.Xi_c = drone2.s(5);
        
        %Trigger
        if (h2>17) %If __________
            Flight_Plan_2.State = 1;
            mySim.start_loop_time2 = mySim.t; %Start timer
            Flight_Plan_2 = update_waypoints(Flight_Plan_2,waypoints_2_to_5,drone2);
        end
        
    % State 1 - Follow Waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_2.State == 1)
        % Calculations
        
        [flag,r,q,c,rho,lambda,Flight_Plan_2] = followWaypoints(Flight_Plan_2,drone2);   
        
        %Output
        drone2.u.Height = 25; 
        drone2.u.AirSpeed = 5;
        drone2.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone2);
        
        
        %Trigger
        if Flight_Plan_2.waypoint_target>Flight_Plan_2.n_waypoints
            
            mySim.start_loop_time2 = mySim.t; %Restart timer
            Flight_Plan_2.State = 2; %otherwise go to circle stage
            Flight_Plan_2 = update_circle(Flight_Plan_2,Drone_Targets(5).points);
        end
        
    %State 2 - do circles until landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_2.State ==2)
        
        flag = 2;
        c = Flight_Plan_2.circle_center;
        rho = Flight_Plan_2.loiter_radius;
        lambda = 1;
        r = 0;
        q = 0;
        
        %Output
        drone2.u.Height = change_altitude(-20*mySim.dt,10,drone2);
        drone2.u.AirSpeed = 5;
        drone2.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone2);
        
        Flight_Plan_2 = count_circles(drone2,Flight_Plan_2,c);
        %Trigger     
        if (Flight_Plan_2.loop_count > 2) 
            Flight_Plan_2.State = Flight_Plan_2.LANDING; 
            mySim.start_loop_time2 = mySim.t; %Restart timer
        end

    %State LANDING - Come in for a landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_2.State == Flight_Plan_2.LANDING)
        %calculations
        mySim.loop_time2 = mySim.t - mySim.start_loop_time2; %Check timer
        
        %Output
        drone2.u.Height = change_altitude(-40*mySim.dt,0,drone2);
        drone2.u.AirSpeed = change_speed(-20*mySim.dt,1,drone2);
        drone2.u.Xi_c = deg2rad(0);
        
        %Trigger
        if (h2<1) %If __________
            Flight_Plan_2.State = Flight_Plan_2.TOUCHDOWN;
            mySim.start_loop_time2 = mySim.t; %Restart timer
        end
        
    
    % State TOUCHDOWN - Wait 10 seconds after landing %%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan_2.State == Flight_Plan_2.TOUCHDOWN)
        %calculations
        mySim.loop_time2 = mySim.t - mySim.start_loop_time2; %Check timer
        
        %Output
        drone2.u.Height = 0;
        drone2.u.AirSpeed = 0;
        drone2.u.Xi_c = deg2rad(0);
        
        
        %Trigger
        if (mySim.loop_time > 10)
            mySim.sim_flag2 = false;
        end
    end
    
    
    %% Collision Avoidance
    
    
    dist_12(i) = norm([pn1;pe1]-[pn2;pe2]); %2D distance    
    coll_12(i) = norm([pn1;pe1;h1]-[pn2;pe2;h2]); %3D distance
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Add Collision Avoidance Here
    
    if (dist_12(i)<10)
       %Check altitude?
       if (abs(h1-h2)<10)
           if (h1<h2)
               drone1.u.Height = drone1.u.Height-10;
               if drone1.u.Height <10
                   drone1.u.Height = 10;
               end
           else
               drone2.u.Height = drone2.u.Height-10;
               if drone2.u.Height <10
                   drone2.u.Height = 10;
               end
           end
       end
        
    end
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
    if ((coll_12(i)<1)) 
        mySim.sim_flag = false; %end simulation if we crash
    end
    
    
    %% Calculate drone dynamics
    
    drone1 = update_drone(mySim.dt,drone1);
    drone1.data.x(:,i) = drone1.s;
    drone1.data.u(:,i) = [drone1.u.Height,drone1.u.AirSpeed,drone1.u.RollAngle,drone1.u.Xi_c];
    
    drone2 = update_drone(mySim.dt,drone2);
    drone2.data.x(:,i) = drone2.s;
    drone2.data.u(:,i) = [drone2.u.Height,drone2.u.AirSpeed,drone2.u.RollAngle,drone2.u.Xi_c];
    
        
    mySim.t = mySim.t+mySim.dt;
    i = i+1;
    
    %Only stop simulation if both flight plans end
    if (mySim.sim_flag1 == false)&&(mySim.sim_flag2 == false)
        mySim.sim_flag = false;
    end
    
    %end simulation if we run out of space
    if (i>=mySim.idx_end)
        mySim.sim_flag = false;
    end
end


t = 0:mySim.dt:mySim.time_end;
i = i-1;


%% plot everything

%plot_3D_flight_path(drone,i,160);

I = imread('city_map2.jpg');
J = flipud(I);
figure;
imagesc(J); hold on;
set(gca,'YDir','normal');
grid on;
axis equal;
xlim([1 1463])
ylim([1 920]) 

plot_Obs(Obstacles);
plot_DroneTargets(Drone_Targets);
plot_Voronoi_Map(Obs);    
plot(drone1.data.x(2,1:i),drone1.data.x(1,1:i),'b','LineWidth',5);
hold on;
str = "Total Time: "+(i*mySim.dt)/60 + " minutes";
title({'No Crashing',str})
xlabel('Dont change the waypoints or the airspeed');
ylabel('Play around with altitude or detection');
%Add title and legend to this figure
plot(drone2.data.x(2,1:i),drone2.data.x(1,1:i),'r','LineWidth',5);

if MOVIE
filename = "Final_Delivery_UTM.mp4";
movie_speedup_rate = 360;
fps = 20;
resolution = 720;
tic
makeMovie_City(filename,drone1.data.x(:,1:i),drone2.data.x(:,1:i),Drone_Targets,Obstacles,movie_speedup_rate,fps,resolution);
toc
end


%% Did they crash?

if CRASH_PLOT
figure;
plot(dist_12(1:i));
hold on;
plot([1,i],[10,10],'r--');
title('Lateral Distance');
xlabel('Time (s)');
ylabel('Distance (m)');
legend('Distance between Drone 1 and 2','Threshold');

figure;
plot(coll_12(1:i));
hold on;
plot([1,i],[10,10],'r--');
title('Collision Distance');
xlabel('Time (s)');
ylabel('Distance (m)');
legend('Distance between Drones 1 and 2','Threshold');

end % end CRASHPLOT


end %end RUN SIM

%% Helper Functions

%Implement P controller for calculating a roll angle from a given heading
function RollAngle = calc_RollAngle(drone)
Xi = drone.s(5);
Xi_c = drone.u.Xi_c;

%Keep Xi_c between 0 and 360 degrees
Xi_c = wrapTo2Pi(Xi_c);

%Why do we have this condition? hint: what happens if we're heading
%north-east and we want to turn north-west
if Xi_c-Xi < -pi
    Xi_c = Xi_c + 2*pi;
end
if Xi_c-Xi > pi
    Xi_c = Xi_c - 2*pi;
end

RollAngle = drone.param.bX*(Xi_c-Xi);

%Why do we have this here?

if RollAngle > drone.param.max_bank
    RollAngle = drone.param.max_bank;
end
if RollAngle < -drone.param.max_bank
    RollAngle = -drone.param.max_bank;
end
end

%Initialize the Simulation
function mySim = initialize_simulation()
%initialize everything
mySim.t = 0;
mySim.time_end = 3000;
mySim.dt = 0.02;
mySim.idx_end = mySim.time_end/mySim.dt;

mySim.start_loop_time = 10000;
mySim.loop_time = 0;

mySim.sim_flag = true;
end

%Initialize a Flight Plan
function [Flight_Plan] = initialize_flight_plan()
Flight_Plan.State = 0;
Flight_Plan.waypoints = [300, 400;
    300, -200;
    100, -200;
    100,300];

Flight_Plan.waypoint_target = 1;
Flight_Plan.n_waypoints = length(Flight_Plan.waypoints);
Flight_Plan.w0 = 0;
Flight_Plan.w1 = 0;
Flight_Plan.w2 = 0;
Flight_Plan.line_orbit = 0;
Flight_Plan.last_line = 0;
Flight_Plan.turn_radius = 20;
Flight_Plan.loiter_radius = 20;

Flight_Plan.start_circle_count = 0;
Flight_Plan.circle_center = [0;0];
Flight_Plan.loop_count = 0;
Flight_Plan.start_entry_angle = 0;
Flight_Plan.complete_flag = false;

Flight_Plan.waypoint_state = 1;
Flight_Plan.line = 1;
Flight_Plan.orbit = 2;

Flight_Plan.LANDING = 777;
Flight_Plan.FLARE = 888;
Flight_Plan.TOUCHDOWN = 999;
end

%Initialize drone
function drone = initialize_drone()

drone.airframe = fixedwing;
drone.airframe.Configuration.FlightPathAngleLimits = [-.55,0.55];


%Drone structure: p_north, p_east, altitude, airspeed, course, pitch, roll, roll_d
drone.s = state(drone.airframe);
drone.s(4) = 3;
drone.u = control(drone.airframe);
drone.e = environment(drone.airframe);

%u = height, AirSpeed, RollAngle
drone.u.Height = 10;
drone.u.AirSpeed = 3;
drone.u.RollAngle = 0;
drone.u.Xi_c = 0;

%Parameters
drone.param.bX = 0.75;
drone.param.max_bank = deg2rad(65);
drone.param.k_orbit = 3;
drone.param.k_line = 0.1;
drone.param.Xinf = deg2rad(70);
end

%Update drone
function drone = update_drone(dt,drone)


% Implement P control for bank angle
drone.u.RollAngle = calc_RollAngle(drone);

%update the aircraft
sdot = derivative(drone.airframe,drone.s,drone.u,drone.e);
drone.s = drone.s+sdot*dt;
drone.s(5) = wrapTo2Pi(drone.s(5));


end

%Update Waypoints
function [Flight_Plan] = update_waypoints(Flight_Plan,waypoints,drone)
Flight_Plan.waypoints = waypoints;
Flight_Plan.n_waypoints = length(Flight_Plan.waypoints);
Flight_Plan.waypoint_target = 1;
Flight_Plan.w0 = [drone.s(1);drone.s(2)];
Flight_Plan.flag = 1;
Flight_Plan.waypoint_state = 1;

end

function makeMovie_City(filename,drone1,drone2,targets,obstacles,scale,fps,resolution)

    steps = length(drone1);
    tile_name = "Drone Delivery Solution";
    
    movie_length = floor(steps/scale);
    
    v = VideoWriter(filename,'MPEG-4');
    v.Quality = 95;
    v.FrameRate = fps;
    open(v);

    hres = round(1.6*resolution/2)*2;    
    figure('position', [70, 70, hres, resolution]);
    I = imread('city_map2.jpg');
    J = flipud(I);
    
    
    
    for i = 1:movie_length
        clf('reset');
        imagesc(J); hold on;
        set(gca,'YDir','normal');
        grid on;
        axis equal;
        xlim([1 1463])
        ylim([1 920]) 
        plot_DroneTargets(targets);
        hold on;
        plot_Obs(obstacles);
        linestart = max([1 i-10]);
        plot(drone1(2,i*scale),drone1(1,i*scale),'wo','MarkerSize',12,'MarkerFaceColor','r','LineWidth',3);
        plot(drone1(2,linestart*scale:i*scale),drone1(1,linestart*scale:i*scale),'-w','LineWidth',3);
        
        plot(drone2(2,i*scale),drone2(1,i*scale),'wo','MarkerSize',12,'MarkerFaceColor','b','LineWidth',3);
        plot(drone2(2,linestart*scale:i*scale),drone2(1,linestart*scale:i*scale),'-w','LineWidth',3);
        
        title(tile_name);
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    close(v);

end


function new_alt = change_altitude(speed,floor,drone)
    new_alt = drone.s(3)+speed;
    if (new_alt < floor)
        new_alt = floor;
    end
end

function new_speed = change_speed(speed,floor,drone)
    new_speed = drone.s(4)+speed;
    if (new_speed < floor)
        new_speed = floor;
    end
end

function [flag,r,q,c,rho,lambda,Flight_Plan] = followWaypoints(Flight_Plan,drone)
    pn = drone.s(1);
    pe = drone.s(2);
    flag = 0;
    r = 0;
    q = 0;
    c = 0;
    rho = Flight_Plan.turn_radius;
    lambda = 0;
    
    % If we've already gone to the first waypoint
    if (Flight_Plan.waypoint_target > 1)
        Flight_Plan.w0 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target-1,1);...
            Flight_Plan.waypoints(Flight_Plan.waypoint_target-1,2)];
    end
    Flight_Plan.w1 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target,1);...
        Flight_Plan.waypoints(Flight_Plan.waypoint_target,2)];

    % If there is a next waypoint for us to go to
    if (Flight_Plan.waypoint_target < Flight_Plan.n_waypoints)
        Flight_Plan.w2 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target+1,1);...
            Flight_Plan.waypoints(Flight_Plan.waypoint_target+1,2)];
    else
        Flight_Plan.w2 = [0;0];
    end

    w_diff = Flight_Plan.w1-Flight_Plan.w0;
    q0 = w_diff/norm(w_diff);
    w_next = Flight_Plan.w2-Flight_Plan.w1;
    q1 = w_next/norm(w_next);


    if (Flight_Plan.waypoint_target < Flight_Plan.n_waypoints)
        Q = acos(-q0'*q1);
    else
        Q = 1.5708;
    end

    if Flight_Plan.waypoint_state == 1 %we do lines
        flag = 1;
        r = Flight_Plan.w0;
        q = q0;
        z = Flight_Plan.w1-(Flight_Plan.turn_radius/(tan(Q/2)))*q0;
        score = ([pn;pe]-z)'*q0;
        if score >= 0
            Flight_Plan.waypoint_state = 2;
            if (Flight_Plan.waypoint_target == Flight_Plan.n_waypoints)
                Flight_Plan.waypoint_target = Flight_Plan.waypoint_target+1;
            end
        end
    elseif Flight_Plan.waypoint_state == 2 %we do circle
        flag = 2;
        c = Flight_Plan.w1-(Flight_Plan.turn_radius/sin(Q/2))*((q0-q1)/norm(q0-q1));
        lambda = sign(q0(1)*q1(2)-q0(2)*q1(1));
        z = Flight_Plan.w1+(Flight_Plan.turn_radius/tan(Q/2))*q1;       
        score = ([pn;pe]-z)'*q1;
        if score >=0
            Flight_Plan.waypoint_target = Flight_Plan.waypoint_target+1;
            Flight_Plan.waypoint_state = 1;
        end
    end
end

function Aircraft_Course = calculateCommand(flag,r,q,c,rho,lambda,drone)
    pn = drone.s(1);
    pe = drone.s(2);
    Xi = drone.s(5);
    %Calculate Flight Commands
    % Set flag = 1 for flying a line
    % Set flag = 2 for flying a circle
    % Set c = [circle north, circle east] to set circle location
    if flag == 1
        Course_d = atan2(q(2),q(1));

        %should we turn left or right
        if (Course_d-Xi) < - pi
            Course_d = Course_d + 2*pi;
        end
        if (Course_d-Xi) > pi
            Course_d = Course_d - 2*pi;
        end

        epy = -sin(Course_d)*(pn-r(1))+ cos(Course_d)*(pe-r(2));

        Aircraft_Course = -drone.param.Xinf*(2/pi)*atan(drone.param.k_line*epy)+Course_d;

    elseif flag == 2
        cn = c(1);
        ce = c(2);

        d = sqrt((pn-cn)^2+(pe-ce)^2);
        psi = atan2(pe-ce,pn-cn);
        if (psi-Xi)<-pi
            psi=psi+2*pi;
        end
        if (psi-Xi)>pi
            psi=psi-2*pi;
        end
        Aircraft_Course = psi+lambda*(pi/2+atan(drone.param.k_orbit*((d-rho)/rho)));
    end
end

function Flight_Plan = update_circle(Flight_Plan,c)
    cn = c(1);
    ce = c(2);
    Flight_Plan.circle_center = [cn,ce];
    Flight_Plan.loop_count = 0;
    Flight_Plan.start_circle_count = 0;
end

function Flight_Plan = count_circles(drone,Flight_Plan,c)
    pe = drone.s(2);
    pn = drone.s(1);
    cn = c(1);
    ce = c(2);

    d = sqrt((pn-cn)^2+(pe-ce)^2);
    psi = atan2(pe-ce,pn-cn);
    
    if (Flight_Plan.start_circle_count == 0)
        if (abs(Flight_Plan.loiter_radius-d)<10)
            Flight_Plan.start_entry_angle = psi;
            Flight_Plan.start_circle_count = 1;
            Flight_Plan.loop_count = 0;
            Flight_Plan.loop_flip = 0;
        end
    end
    if (Flight_Plan.start_circle_count == 1)
        circle_amount = wrapTo2Pi(psi-Flight_Plan.start_entry_angle);
        if (Flight_Plan.loop_flip == 0)
            if circle_amount > 6.2657
                Flight_Plan.loop_flip = 1;
            end
        end
        if (Flight_Plan.loop_flip == 1)
            if circle_amount < 0.175
                Flight_Plan.loop_count = Flight_Plan.loop_count + 1;
                Flight_Plan.loop_flip = 0;
            end
        end      
    end
end

function plot_Voronoi_Map(Obs)

    obs_x = Obs(:,2);
    obs_y = Obs(:,1);

    [vx,vy] = voronoi(obs_x,obs_y);

    plot(vx,vy,'b-'); hold on;
    xlim([1 1463])
    ylim([1 920])   
end


function [v,G] = Generate_Voronoi(Obs, tolerance)
    X = [Obs(:,2),Obs(:,1)]; %Revert to (East,North) for voronoi calculations
    [v,c] = voronoin(X);
    G = graph;
    for i = 1:length(c)
        line = c{i};
        s = line;
        t = circshift(line,1);
        G = addedge(G,s,t);
    end

    nuEd = numedges(G);
    duplicate_idx = 2:2:nuEd;

    G = rmedge(G,duplicate_idx);

    list_of_edges = G.Edges;

    for i = 1:height(list_of_edges)
        edge1 = list_of_edges{i,1};
        V1 = v(edge1(1),:);
        V2 = v(edge1(2),:);
        if (sum(isinf(V1))>=1)
            V1 = [8000 8000];
        end
        if (sum(isinf(V2))>=1)
            V2 = [8000 8000];
        end

        %find distance to nearest collision point
        Dp_min = 1000;
        for p = 1:length(X)
            point = X(p,:);

            sigma_Star = ((V1-point)*(V1-V2)')/(norm(V1-V2)^2);
            if (sigma_Star<0)
                Dp = norm(point-V1);
            elseif (sigma_Star>1)
                Dp = norm(point-V2);
            else
                Dp = sqrt((norm(point-V1)^2)- (((V1-point)*(V1-V2)'))^2/(norm(V1-V2)^2));
            end
            if (Dp < Dp_min)
                Dp_min = real(Dp);
            end
        end
        distances(i) = norm(V1-V2);
        if (distances(i) > 5000)
            distances(i) = 5000;
        end
        Dist2Obs(i) = Dp_min;
        if (Dist2Obs(i) < 25)
            distances(i) = 5000;
        end

    end
    weights = 2*distances/max(distances)+tolerance./Dist2Obs;

    G.Edges.Weight = weights';

end

function [waypoints_path,waypoints_dist] = Generate_Best_Path_Around_Obstacles(G,v,starting_point,ending_point)
% Generate_Best_Path_Around_Obstacles - generates a shortest path algorithm
% using Voronoi Tesselation.
%
% Inputs: 
%    G - graph
%    v - vertices
%    starting_point - coordinates of starting location (North, East)
%    ending_point - coordinates of ending location (North, East)
%    tolerance - tolerance factor (0-1), relates to how close (0) or far (1) path
%                should get to obstacle
% Outputs:
%    waypoints_path - list of waypoints to travel (North, East)
%    waypoints_dist - total distance of waypoint path


    
    %Find closest node from origin
    min_dist = 1000;
    wp_start = 0;
    for i = 1:length(v)

        dist = norm(v(i,:)-circshift(starting_point,1));
        if (isinf(dist))
            dist = 100000;
        end
        if dist<min_dist
            wp_start = i;
            min_dist=dist;
        end
    end

    

    %Find closest node to end
    min_dist = 1000;
    wp_end = 0;
    for i = 1:length(v)

        dist = norm(v(i,:)-circshift(ending_point,1));
        if (isinf(dist))
            dist = 100000;
        end
        if dist<min_dist
            wp_end = i;
            min_dist=dist;
        end
    end

    [P,d] = shortestpath(G,wp_start,wp_end);

    Pwaypoints = v(P,:);

    waypoints_path = circshift(Pwaypoints,1,2); %Back to (North,East)
    waypoints_path = cat(1,starting_point,waypoints_path,ending_point);
    waypoints_dist = d;
    
    %fix for nodes too close together
    j = 1;
    k = length(waypoints_path);
    idx = [];
    for i = 2:k
       next_way_dist = norm(waypoints_path(i,:)-waypoints_path(i-1,:));
       if next_way_dist < 4
           idx(j) = i;
           j = j+1;
       end
    end
    if ~isempty(idx)
        waypoints_path(idx,:) = [];
    end
    
    waypoints_path = waypoints_path(3:end,:); %Skip first 2 waypoints as flight targets
    
end

function point_list = BuildPoints(Corners,point_distance)

    Obj_size = length(Corners);

    k = 1;
    for i = 1:Obj_size
       next_i = mod(i,Obj_size)+1;
       w0 = Corners(i,:);
       w1 = Corners(next_i,:);
       pt_diff = w1-w0;
       dist = norm(pt_diff);
       theta(i) = atan2(pt_diff(1),pt_diff(2));
       points(i) = floor(dist/point_distance);
       north_points(k) = w0(1);
       east_points(k) = w0(2);
       k = k+1;
       for j = 1:(points(i)-1)
           north_points(k) = point_distance*sin(theta(i))+north_points(k-1);
           east_points(k) = point_distance*cos(theta(i))+east_points(k-1);
           k = k+1;
       end
    end

    point_list = cat(2,north_points',east_points');
end

function plotPointList(point_list)

    point_count = length(point_list);
    for i=1:point_count
        m = mod(i,point_count)+1;
        plot([point_list(i,2),point_list(m,2)],[point_list(i,1),point_list(m,1)],'r-','LineWidth',4); hold on;
    end
end

function plotTrees(point_list)

    plot(point_list(:,2),point_list(:,1),'rh','MarkerSize',12,'MarkerFaceColor','r');
end

function Obs = build_total_points(obstacles)
    num_buildings = numel(obstacles.Buildings);
    Obs = cat(1,obstacles.Buildings(1).points,obstacles.Buildings(2).points);
    if (num_buildings>2)
        for i = 1:(num_buildings-2)
            Obs = cat(1,Obs,obstacles.Buildings(i+2).points);
        end
    end
    Obs = cat(1,Obs,obstacles.Trees);
    Obs = cat(1,Obs,obstacles.GeoFence.points);

end

function plot_Obs(obstacles)
    num_buildings = numel(obstacles.Buildings);
    for i = 1:num_buildings
        plotPointList(obstacles.Buildings(i).points);
    end
    plotTrees(obstacles.Trees);
    plotPointList(obstacles.GeoFence.points);
end

function plot_DroneTargets(Targets)
    targs = numel(Targets);
    for i = 1:targs
        if (strcmp(Targets(i).name,'Launch Point'))
            plot(Targets(i).points(2), Targets(i).points(1),'go','MarkerSize',10,'MarkerFaceColor','g');
            str = "LP - "+i;
            text(Targets(i).points(2)+20, Targets(i).points(1),str,'Color','green','FontSize',14,'FontWeight','bold');
        end
        if (strcmp(Targets(i).name,'Delivery'))
            plot(Targets(i).points(2), Targets(i).points(1),'yo','MarkerSize',10,'MarkerFaceColor','y');
            str = "DP - "+i;
            text(Targets(i).points(2)+20, Targets(i).points(1),str,'Color','magenta','FontSize',14,'FontWeight','bold');
        end
    end
end

function Obstacles = Create_Obstacle_Map()
    Obstacles = struct;

    Obstacles.Buildings(1).corners = [513, 403;
                                     513, 90;
                                     616, 90;
                                     616, 403];
    Obstacles.Buildings(2).corners = [475, 230;
                                     475, 420;
                                     375, 420;
                                     375, 230];
    Obstacles.Buildings(3).corners = [280, 989;
                                      280, 1049;
                                      174, 1049;
                                      174, 989];
    Obstacles.Buildings(4).corners = [282, 253;
                                      282, 470;
                                      243, 470;
                                      243, 270;
                                      232, 260;
                                      167, 260;
                                      167, 220;
                                      250, 220];
    Obstacles.Buildings(5).corners = [854, 396;
                                      854, 472;
                                      775, 472;
                                      775, 396];

    Obstacles.Trees = [516, 663;
                       486, 748;
                       422, 898;
                       223, 650;
                       666, 1226;
                       241, 1117;
                       110, 470;
                       501, 551;
                       854, 491;
                       702, 498;
                       655, 933;
                       805, 575;
                       367, 1230;
                       513, 1084;
                       478, 1292];

    Obstacles.GeoFence.corners = [920,1;
                          920, 1463;
                          1, 1463;
                          1,1];

    for i = 1:numel(Obstacles.Buildings)
        Obstacles.Buildings(i).points = BuildPoints(Obstacles.Buildings(i).corners,25);
    end
    Obstacles.GeoFence.points = BuildPoints(Obstacles.GeoFence.corners,50);

end

function Drone_Targets = Create_Drone_Targets()
    %coordinates in North, East

    Drone_Targets = struct;
    Drone_Targets(1).name = 'Launch Point';
    Drone_Targets(1).points = [810,817];
    Drone_Targets(2).name = 'Launch Point';
    Drone_Targets(2).points = [710,817];
    Drone_Targets(3).name = 'Delivery';
    Drone_Targets(3).points = [330,1304];
    Drone_Targets(4).name = 'Delivery';
    Drone_Targets(4).points = [577, 1328];
    Drone_Targets(5).name = 'Delivery';
    Drone_Targets(5).points = [247, 150];
    Drone_Targets(6).name = 'Delivery';
    Drone_Targets(6).points = [785,495];
    Drone_Targets(7).name = 'Delivery';
    Drone_Targets(7).points = [328,798];
    Drone_Targets(8).name = 'Extra Credit - Add and Display a fifth delivery point';
    Drone_Targets(8).points = [];
    
end
