clc;
clear('all');
% rng('default');
% rng(1);
warning('off','all');
format long;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PARAMETERS = {};
PARAMETERS.CREATE_PDF = false;
PARAMETERS.SAMPLING_TIME = 1e-1;
PARAMETERS.TOTAL_TIME = 1200;
PARAMETERS.NORRBIN_A1 = 13.17;
PARAMETERS.NORRBIN_A2 = 16323.46;
PARAMETERS.K = 0.21;
PARAMETERS.T = 107.76;
PARAMETERS.PLOT_FONT_SIZE = 12.0;
PARAMETERS.LEGEND_FONT_SIZE = 8.0;
PARAMETERS.SIMULATION_TYPE = 1; % 1->Straight line path; 2->Sinusoidal path
PARAMETERS.DISTURBANCE_TYPE = 0; % 0->No disturbance; 1->Train of sinusoidal multiple frequencies disturbance 
if PARAMETERS.DISTURBANCE_TYPE == 0
    PARAMETERS.DISTURBANCE_AMPLITUDE = 0.0;
    PARAMETERS.DISTURBANCE_FREQUENCY = 0.0;
elseif PARAMETERS.DISTURBANCE_TYPE == 1
    PARAMETERS.DISTURBANCE_AMPLITUDE = 0.25e-3;
    PARAMETERS.DISTURBANCE_FREQUENCY = 1.12*pi/50;
end    
PARAMETERS.REFERENCE_AMPLITUDE = 50*pi/180.0;
PARAMETERS.REFERENCE_FREQUENCY = 2*pi/600;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROL PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PARAMETERS.SETTLING_TIME = 150;
PARAMETERS.NU = 1.0e-4;
PARAMETERS.OMEGA_C = 1;
PARAMETERS.CONTROL_ALFA_INITIAL = -1.25*log(PARAMETERS.NU/PARAMETERS.REFERENCE_AMPLITUDE)/PARAMETERS.SETTLING_TIME;
PARAMETERS.CONTROL_ALFA_MIN = 1*PARAMETERS.CONTROL_ALFA_INITIAL;
PARAMETERS.CONTROL_ALFA_MAX = 5*PARAMETERS.CONTROL_ALFA_INITIAL + 5*PARAMETERS.DISTURBANCE_FREQUENCY;
PARAMETERS.CONTROL_YAW_RATE_MAX = 0.70*pi/180;
PARAMETERS.CONTROL_LAMBDA_MIN = 1.0*PARAMETERS.CONTROL_YAW_RATE_MAX/PARAMETERS.REFERENCE_AMPLITUDE;
PARAMETERS.CONTROL_LAMBDA_MAX = 2.0*PARAMETERS.CONTROL_LAMBDA_MIN;
PARAMETERS.CONTROL_POWER_GAIN_MIN = 0.8;%1.7443;%1.76952;
PARAMETERS.CONTROL_POWER_GAIN_MAX = 1.685;%3*PARAMETERS.CONTROL_POWER_GAIN_MIN;
PARAMETERS.SIMULATION_COUNTER = 0;
PARAMETERS.SIMULATION_INITIAL_POSITIONS = 1; 
PARAMETERS


for simulation_counter = 1:PARAMETERS.SIMULATION_INITIAL_POSITIONS
%     simulation_counter
    % Run simularion
    if PARAMETERS.SIMULATION_TYPE  == 1
        PARAMETERS.initial_state = [0.0, 0.0]; % yaw, r
    else    
        if PARAMETERS.SIMULATION_INITIAL_POSITIONS == 1
            PARAMETERS.initial_state = [10.0*pi/180.0, 0]; % yaw, r   
        else
            yaw = 50*pi*(rand(1)-0.5)/180.0
            dot_yaw = 5*pi*(rand(1)-0.5)/180.0
            PARAMETERS.initial_state = [yaw, dot_yaw]; % yaw, r   
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    SIMULATION_DATA = run_simulation(PARAMETERS);
    plot_simulation(SIMULATION_DATA, PARAMETERS);
    PARAMETERS.SIMULATION_COUNTER =+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run simulation
function SIMULATION_DATA = run_simulation(PARAMETERS)
    % Simulation time
    simulation_time = 0:PARAMETERS.SAMPLING_TIME:PARAMETERS.TOTAL_TIME-PARAMETERS.SAMPLING_TIME;
    simulation_steps = size(simulation_time, 2) + 1;
    
    % Prepare simulation data
    SIMULATION_DATA = {};
    SIMULATION_DATA.data = zeros(simulation_steps, 5); 
        
    % Nonlinear Improved Concise Backstepping Control
    SIMULATION_DATA.NICB = {};
    SIMULATION_DATA.NICB.data = zeros(simulation_steps, 10); 
    SIMULATION_DATA.NICB.yaw = PARAMETERS.initial_state(1);
    SIMULATION_DATA.NICB.yaw_rate = PARAMETERS.initial_state(2);
    SIMULATION_DATA.NICB.int_nicb_error = 0.0;
    SIMULATION_DATA.NICB.MAE = 0.0;
    SIMULATION_DATA.NICB.MIA = 0.0;
    SIMULATION_DATA.NICB.MTV = 0.0;
    SIMULATION_DATA.NICB.control = 0.0;PARAMETERS.SIMULATION_COUNTER 
        
    % Synergetic Control
    SIMULATION_DATA.SYN = {};
    SIMULATION_DATA.SYN.data = zeros(simulation_steps, 10); 
    SIMULATION_DATA.SYN.yaw = PARAMETERS.initial_state(1);
    SIMULATION_DATA.SYN.yaw_rate = PARAMETERS.initial_state(2);
    SIMULATION_DATA.SYN.int_syn_yaw_error = 0.0;
    SIMULATION_DATA.SYN.MAE = 0.0;
    SIMULATION_DATA.SYN.MIA = 0.0;
    SIMULATION_DATA.SYN.MTV = 0.0;
    SIMULATION_DATA.SYN.control = 0.0;
    
    % AISM
    SIMULATION_DATA.AISM = {};
    SIMULATION_DATA.AISM.data = zeros(simulation_steps, 12); 
    SIMULATION_DATA.AISM.yaw = PARAMETERS.initial_state(1);
    SIMULATION_DATA.AISM.yaw_rate = PARAMETERS.initial_state(2);
    SIMULATION_DATA.AISM.int_aism_s = 0.0;
    SIMULATION_DATA.AISM.int_dot_aism_alfa = PARAMETERS.CONTROL_ALFA_INITIAL;
    SIMULATION_DATA.AISM.MAE = 0.0;
    SIMULATION_DATA.AISM.MIA = 0.0;
    SIMULATION_DATA.AISM.MTV = 0.0;
    SIMULATION_DATA.AISM.control = 0.0;    
    
    % Run simulation
    simulation_time = 0.0;
    for simulation_step = 1:simulation_steps        

        %------------- Disturbance -------------%  
        if PARAMETERS.DISTURBANCE_TYPE == 0
             disturbance = PARAMETERS.DISTURBANCE_AMPLITUDE;
        elseif PARAMETERS.DISTURBANCE_TYPE == 1
            disturbance = PARAMETERS.DISTURBANCE_AMPLITUDE*cos(PARAMETERS.DISTURBANCE_FREQUENCY*simulation_time);
            disturbance = disturbance + 0.83*PARAMETERS.DISTURBANCE_AMPLITUDE*sin(3.29*PARAMETERS.DISTURBANCE_FREQUENCY*simulation_time - 0.14);
            disturbance = disturbance + 1.23*PARAMETERS.DISTURBANCE_AMPLITUDE*cos(8.12*PARAMETERS.DISTURBANCE_FREQUENCY*simulation_time + 0.26);          
            disturbance = disturbance + 0.65*PARAMETERS.DISTURBANCE_AMPLITUDE*sin(1.37*PARAMETERS.DISTURBANCE_FREQUENCY*simulation_time + 0.36)*exp(cos(2.21*PARAMETERS.DISTURBANCE_FREQUENCY*simulation_time + 0.13));            
        end 
    
        %------------- Reference -------------% 
        if PARAMETERS.SIMULATION_TYPE  == 1
            yaw_ref = PARAMETERS.REFERENCE_AMPLITUDE;
            yaw_rate_ref = 0.0;
            dot_yaw_rate_ref = 0.0;
        else            
            yaw_ref = PARAMETERS.REFERENCE_AMPLITUDE*sin(PARAMETERS.REFERENCE_FREQUENCY*simulation_time);
            yaw_rate_ref = PARAMETERS.REFERENCE_AMPLITUDE*PARAMETERS.REFERENCE_FREQUENCY*cos(PARAMETERS.REFERENCE_FREQUENCY*simulation_time);
            dot_yaw_rate_ref = -PARAMETERS.REFERENCE_AMPLITUDE*PARAMETERS.REFERENCE_FREQUENCY*PARAMETERS.REFERENCE_FREQUENCY*sin(PARAMETERS.REFERENCE_FREQUENCY*simulation_time);            
        end
        
        %------------- Save global data -------------% 
        SIMULATION_DATA.data(simulation_step, 1) = simulation_time;
        SIMULATION_DATA.data(simulation_step, 2) = yaw_ref;
        SIMULATION_DATA.data(simulation_step, 3) = yaw_rate_ref;
        SIMULATION_DATA.data(simulation_step, 4) = dot_yaw_rate_ref;
        SIMULATION_DATA.data(simulation_step, 5) = disturbance;
        
        if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%% Nonlinear Improved Concise Backstepping Control of Course
            %%%% Keeping for Ships 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %------------- Control -------------% 
            nicb_k1 = 0.0017;
            nicb_w = 0.6;
            nicb_error = SIMULATION_DATA.NICB.yaw - yaw_ref;
            dot_nicb_error = SIMULATION_DATA.NICB.yaw_rate - yaw_rate_ref;
            [nicb_f, nicb_b] = ship_dynamics_f_b(SIMULATION_DATA.NICB.yaw_rate, PARAMETERS);
            nicb_control_prev = SIMULATION_DATA.NICB.control;
            SIMULATION_DATA.NICB.control = (1.0/nicb_b)*(nicb_f - nicb_k1*atan(nicb_w*nicb_error));      
             
            %------------- Save data -------------% 
            SIMULATION_DATA.NICB.data(simulation_step, 1) = SIMULATION_DATA.NICB.yaw;
            SIMULATION_DATA.NICB.data(simulation_step, 2) = SIMULATION_DATA.NICB.yaw_rate;
            SIMULATION_DATA.NICB.data(simulation_step, 3) = SIMULATION_DATA.NICB.control;
            SIMULATION_DATA.NICB.data(simulation_step, 4) = SIMULATION_DATA.NICB.MAE;
            SIMULATION_DATA.NICB.data(simulation_step, 5) = SIMULATION_DATA.NICB.MIA;
            SIMULATION_DATA.NICB.data(simulation_step, 6) = SIMULATION_DATA.NICB.MTV;
            SIMULATION_DATA.NICB.data(simulation_step, 7) = nicb_error;
            SIMULATION_DATA.NICB.data(simulation_step, 8) = dot_nicb_error;
            
            %------------- Ship dynamics -------------% 
            [SIMULATION_DATA.NICB.yaw, SIMULATION_DATA.NICB.yaw_rate] = ship_dynamics(SIMULATION_DATA.NICB.yaw, SIMULATION_DATA.NICB.yaw_rate, SIMULATION_DATA.NICB.control, disturbance, PARAMETERS);
           
            %------------- Update states -------------% 
            SIMULATION_DATA.NICB.int_e1 = SIMULATION_DATA.NICB.int_nicb_error + nicb_error*PARAMETERS.SAMPLING_TIME; 
            SIMULATION_DATA.NICB.MAE = (SIMULATION_DATA.NICB.MAE + abs(nicb_error)*PARAMETERS.SAMPLING_TIME);        
            SIMULATION_DATA.NICB.MIA = (SIMULATION_DATA.NICB.MIA + abs(SIMULATION_DATA.NICB.control)*PARAMETERS.SAMPLING_TIME);
            if simulation_step > 1
                 SIMULATION_DATA.NICB.MTV = (SIMULATION_DATA.NICB.MTV + abs(SIMULATION_DATA.NICB.control - nicb_control_prev)*PARAMETERS.SAMPLING_TIME);  
            end        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%% Robust integral backstepping and terminal synergetic control of course 
            %%%% keeping for ships
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %------------- Control -------------% 
            syn_a1 = 0.09;
            syn_a2 = 1.8912;
            syn_T1 = 28.0; 
            syn_yaw_error = SIMULATION_DATA.SYN.yaw - yaw_ref;                
            dot_syn_yaw_error = SIMULATION_DATA.SYN.yaw_rate - yaw_rate_ref;
            [syn_f, syn_b] = ship_dynamics_f_b(SIMULATION_DATA.SYN.yaw_rate, PARAMETERS);
            syn_s = syn_a1*syn_yaw_error + syn_a2*dot_syn_yaw_error;
            syn_control_prev = SIMULATION_DATA.SYN.control;
            SIMULATION_DATA.SYN.control = (-1.0/(syn_b*syn_a2))*(syn_a1*SIMULATION_DATA.SYN.yaw_rate + syn_a2*syn_f + (syn_s/syn_T1));    
            
            %------------- Save data -------------% 
            SIMULATION_DATA.SYN.data(simulation_step, 1) = SIMULATION_DATA.SYN.yaw;
            SIMULATION_DATA.SYN.data(simulation_step, 2) = SIMULATION_DATA.SYN.yaw_rate;
            SIMULATION_DATA.SYN.data(simulation_step, 3) = SIMULATION_DATA.SYN.control;
            SIMULATION_DATA.SYN.data(simulation_step, 4) = SIMULATION_DATA.SYN.MAE;
            SIMULATION_DATA.SYN.data(simulation_step, 5) = SIMULATION_DATA.SYN.MIA;
            SIMULATION_DATA.SYN.data(simulation_step, 6) = SIMULATION_DATA.SYN.MTV;
            SIMULATION_DATA.SYN.data(simulation_step, 7) = syn_yaw_error;
            SIMULATION_DATA.SYN.data(simulation_step, 8) = dot_syn_yaw_error;
            
            %------------- Ship dynamics -------------% 
            [SIMULATION_DATA.SYN.yaw, SIMULATION_DATA.SYN.yaw_rate] = ship_dynamics(SIMULATION_DATA.SYN.yaw, SIMULATION_DATA.SYN.yaw_rate, SIMULATION_DATA.SYN.control, disturbance, PARAMETERS);
           
            %------------- Update states -------------% 
            SIMULATION_DATA.SYN.int_e1 = SIMULATION_DATA.SYN.int_syn_yaw_error + syn_yaw_error*PARAMETERS.SAMPLING_TIME;
            SIMULATION_DATA.SYN.MAE = (SIMULATION_DATA.SYN.MAE + abs(syn_yaw_error)*PARAMETERS.SAMPLING_TIME);        
            SIMULATION_DATA.SYN.MIA = (SIMULATION_DATA.SYN.MIA + abs(SIMULATION_DATA.SYN.control)*PARAMETERS.SAMPLING_TIME);
            if simulation_step > 1
                 SIMULATION_DATA.SYN.MTV = (SIMULATION_DATA.SYN.MTV + abs(SIMULATION_DATA.SYN.control - syn_control_prev)*PARAMETERS.SAMPLING_TIME);  
            end  
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Adaptive Integral Sliding Mode Control
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        aism_yaw_error = SIMULATION_DATA.AISM.yaw - yaw_ref;               
        dot_aism_yaw_error = SIMULATION_DATA.AISM.yaw_rate - yaw_rate_ref;       
        [aism_f, aism_b] = ship_dynamics_f_b(SIMULATION_DATA.AISM.yaw_rate, PARAMETERS);
        aism_control_prev = SIMULATION_DATA.AISM.control;        
        aism_lambda = PARAMETERS.CONTROL_LAMBDA_MAX - ((PARAMETERS.CONTROL_LAMBDA_MAX-PARAMETERS.CONTROL_LAMBDA_MIN)/PARAMETERS.REFERENCE_AMPLITUDE)*abs(aism_yaw_error);
        dot_aism_lambda = - ((PARAMETERS.CONTROL_LAMBDA_MAX-PARAMETERS.CONTROL_LAMBDA_MIN)/PARAMETERS.REFERENCE_AMPLITUDE)*sign(aism_yaw_error)*dot_aism_yaw_error;        
        aism_s = dot_aism_yaw_error + aism_lambda*aism_yaw_error; 
        aism_alfa = SIMULATION_DATA.AISM.int_dot_aism_alfa;

        aism_z = aism_s + (aism_alfa/2.0)*SIMULATION_DATA.AISM.int_aism_s;
        aism_power_gain = ((PARAMETERS.CONTROL_POWER_GAIN_MAX - PARAMETERS.CONTROL_POWER_GAIN_MIN)/PARAMETERS.REFERENCE_AMPLITUDE)*abs(aism_yaw_error) + PARAMETERS.CONTROL_POWER_GAIN_MIN; 
        aism_kappa = (1/(PARAMETERS.NU^(1/(aism_power_gain+1))))*aism_alfa/2.0;
        aism_gamma = (aism_alfa^2)/4.0;      
        SIMULATION_DATA.AISM.control = (1.0/aism_b)*(-aism_f + dot_yaw_rate_ref - aism_lambda*dot_aism_yaw_error - dot_aism_lambda*aism_yaw_error - aism_alfa*aism_s - aism_gamma*SIMULATION_DATA.AISM.int_aism_s);
                
        %------------- Update control states -------------% 
        SIMULATION_DATA.AISM.int_aism_s = SIMULATION_DATA.AISM.int_aism_s + aism_s*PARAMETERS.SAMPLING_TIME;
       
        dot_aism_alfa = aism_kappa*sign(aism_z)*(abs(aism_z)^aism_power_gain)*sign(aism_s);
        if abs(aism_z) < PARAMETERS.NU
            dot_aism_alfa = 0;
        end
        aism_alfa = aism_alfa + dot_aism_alfa*PARAMETERS.SAMPLING_TIME;
        if  (aism_alfa < PARAMETERS.CONTROL_ALFA_MIN)
            aism_alfa = PARAMETERS.CONTROL_ALFA_MIN;
        elseif (aism_alfa > PARAMETERS.CONTROL_ALFA_MAX)
            aism_alfa = PARAMETERS.CONTROL_ALFA_MAX;
        end
        SIMULATION_DATA.AISM.int_dot_aism_alfa = aism_alfa;

        %------------- Save data -------------% 
        SIMULATION_DATA.AISM.data(simulation_step, 1) = SIMULATION_DATA.AISM.yaw;
        SIMULATION_DATA.AISM.data(simulation_step, 2) = SIMULATION_DATA.AISM.yaw_rate;
        SIMULATION_DATA.AISM.data(simulation_step, 3) = SIMULATION_DATA.AISM.control;
        SIMULATION_DATA.AISM.data(simulation_step, 4) = SIMULATION_DATA.AISM.MAE;
        SIMULATION_DATA.AISM.data(simulation_step, 5) = SIMULATION_DATA.AISM.MIA;
        SIMULATION_DATA.AISM.data(simulation_step, 6) = SIMULATION_DATA.AISM.MTV;
        SIMULATION_DATA.AISM.data(simulation_step, 7) = aism_yaw_error;
        SIMULATION_DATA.AISM.data(simulation_step, 8) = dot_aism_yaw_error;
        SIMULATION_DATA.AISM.data(simulation_step, 9) = aism_alfa;
        SIMULATION_DATA.AISM.data(simulation_step, 10) = aism_s;
        SIMULATION_DATA.AISM.data(simulation_step, 11) = aism_lambda;
        SIMULATION_DATA.AISM.data(simulation_step, 12) = aism_power_gain;
        SIMULATION_DATA.AISM.data(simulation_step, 13) = aism_kappa;
        
        %------------- Ship dynamics -------------% 
        [SIMULATION_DATA.AISM.yaw, SIMULATION_DATA.AISM.yaw_rate] = ship_dynamics(SIMULATION_DATA.AISM.yaw, SIMULATION_DATA.AISM.yaw_rate, SIMULATION_DATA.AISM.control, disturbance, PARAMETERS);
       
        %------------- Update arror indices -------------% 
        SIMULATION_DATA.AISM.MAE = (SIMULATION_DATA.AISM.MAE + abs(aism_yaw_error)*PARAMETERS.SAMPLING_TIME);        
        SIMULATION_DATA.AISM.MIA = (SIMULATION_DATA.AISM.MIA + abs(SIMULATION_DATA.AISM.control)*PARAMETERS.SAMPLING_TIME);
        if simulation_step > 1
            SIMULATION_DATA.AISM.MTV = (SIMULATION_DATA.AISM.MTV + abs(SIMULATION_DATA.AISM.control - aism_control_prev)*PARAMETERS.SAMPLING_TIME);  
        end  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        % Update time
        simulation_time = simulation_time + PARAMETERS.SAMPLING_TIME;
    end
end

% Ship dynamics
function [yaw_new, yaw_rate_new] = ship_dynamics(yaw, yaw_rate, control, disturbance, PARAMETERS)
    yaw_new = yaw + yaw_rate*PARAMETERS.SAMPLING_TIME;
    [f, b] = ship_dynamics_f_b(yaw_rate, PARAMETERS);
    yaw_rate_new = yaw_rate + (f+(b*control)+disturbance)*PARAMETERS.SAMPLING_TIME;
end

function [f, b] = ship_dynamics_f_b(yaw_rate, PARAMETERS)
    b = PARAMETERS.K / PARAMETERS.T;
    h = PARAMETERS.NORRBIN_A1*yaw_rate + PARAMETERS.NORRBIN_A2*(yaw_rate*yaw_rate*yaw_rate);    
    f = -b*h;
end

% Plot simulation 
function plot_simulation(SIMULATION_DATA, PARAMETERS)
    COUNTER = PARAMETERS.SIMULATION_COUNTER 

    fig1 = figure(1);
    create = false;
    if PARAMETERS.SIMULATION_COUNTER == 0
        create = true;
    end
    if create
        clf(fig1);
    end
    subplot(3,1,1);
    if create
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.data(:,2)*180.0/pi ,'-', 'Color', 'c', 'LineWidth',1.5);
        grid on;
        hold on;
        ylabel('$\psi_{r}(t)$ vs $\psi$(t)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Desired and obtained yaw angle (º)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,1)*180.0/pi ,'-', 'Color', 'k', 'LineWidth',1.0);    
    if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,1)*180.0/pi ,'-', 'Color', 'r', 'LineWidth',1.0);    
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,1)*180.0/pi ,'-', 'Color', 'b', 'LineWidth',1.0);    
    end
    

    if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
        if PARAMETERS.SIMULATION_TYPE == 1
            axes('Position',[.55 .77 .3 .1])
            box on
            data_size = size(SIMULATION_DATA.data(:,1),1) - 1;
            detail_size = floor(100.0/PARAMETERS.SAMPLING_TIME);
            from_to_detail = data_size-detail_size:data_size;    
            plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.data(from_to_detail,2)*180.0/pi ,'-', 'Color', 'c', 'LineWidth',1.5);
            grid on;
            hold on;
            plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.AISM.data(from_to_detail,1)*180.0/pi ,'-', 'Color', 'k',  'LineWidth',1.5);
            if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
                plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.NICB.data(from_to_detail,1)*180.0/pi ,'-', 'Color', 'r', 'LineWidth',1.0); 
                plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.SYN.data(from_to_detail,1)*180.0/pi ,'-', 'Color', 'b',  'LineWidth',1.0);            
            end
        %     title('Detailed view of yaw error at steady-state');
            xlim([PARAMETERS.TOTAL_TIME-detail_size*PARAMETERS.SAMPLING_TIME, PARAMETERS.TOTAL_TIME]);
        end
    end

    subplot(3,1,2);
    if create
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.data(:,3)*180.0/pi ,'-', 'Color', 'c', 'LineWidth',1.5); 
        grid on;
        hold on;
        ylabel('$r_{r}(t)$ vs $r(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Desired and obtained yaw rate (º/s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,2)*180.0/pi ,'-', 'Color', 'k',  'LineWidth',1.0);
    if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,2)*180.0/pi ,'-', 'Color', 'r',  'LineWidth',1.0);   
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,2)*180.0/pi ,'-', 'Color', 'b',  'LineWidth',1.0);
    end    
    
%     if  PARAMETERS.SIMULATION_TYPE == 1
%         legend('Reference','Adaptive Sliding Mode (González-Prieto et al.)', 'Concise Backstepping (Zhang et al.)','Synergetic (Muhammad et al.)','Interpreter','latex','FontSize', PARAMETERS.LEGEND_FONT_SIZE); 
%     end
    
    subplot(3,1,3);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,3)*180.0/pi,'-', 'Color', 'k', 'LineWidth',1.0);    
    if create
        grid on;
        hold on;
        ylabel('$\delta(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Rudder angle', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end    
    if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,3)*180.0/pi,'-', 'Color', 'r', 'LineWidth',1.0);
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,3)*180.0/pi,'-', 'Color', 'b', 'LineWidth',1.0);
    end
    if PARAMETERS.SIMULATION_INITIAL_POSITIONS == 1
        if PARAMETERS.DISTURBANCE_TYPE == 0
            axes('Position',[.30 .20 .4 .1])
            box on
            detail_size = floor(100.0/PARAMETERS.SAMPLING_TIME);
            from_to_detail = 1:detail_size; 
            plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.AISM.data(from_to_detail,3)*180.0/pi,'-', 'Color', 'k', 'LineWidth',1.0);
            grid on;
            hold on;
            if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
                plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.NICB.data(from_to_detail,3)*180.0/pi,'-', 'Color', 'r', 'LineWidth',1.0);
                plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.SYN.data(from_to_detail,3)*180.0/pi,'-', 'Color', 'b', 'LineWidth',1.0);
            end
            xlim([0, detail_size*PARAMETERS.SAMPLING_TIME]);
        end
    end  
    disp('------ Nonlinear Improved Concise Backstepping Control -------');
    MAE = SIMULATION_DATA.NICB.MAE/PARAMETERS.TOTAL_TIME
    MIA = SIMULATION_DATA.NICB.MIA/PARAMETERS.TOTAL_TIME
    MTV = SIMULATION_DATA.NICB.MTV/PARAMETERS.TOTAL_TIME
    
    disp('------ Synergetic Control -------');
    MAE = SIMULATION_DATA.SYN.MAE/PARAMETERS.TOTAL_TIME
    MIA = SIMULATION_DATA.SYN.MIA/PARAMETERS.TOTAL_TIME
    MTV = SIMULATION_DATA.SYN.MTV/PARAMETERS.TOTAL_TIME
   
    disp('------ AISM -------');
    MAE = SIMULATION_DATA.AISM.MAE/PARAMETERS.TOTAL_TIME
    MIA = SIMULATION_DATA.AISM.MIA/PARAMETERS.TOTAL_TIME
    MTV = SIMULATION_DATA.AISM.MTV/PARAMETERS.TOTAL_TIME
   
    fig2 = figure(2);
    if create
        clf(fig2);
    end
    subplot(3,1,1);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,4)/PARAMETERS.TOTAL_TIME,'-', 'Color', 'k', 'LineWidth',1.0);       
    if create
        grid on;
        hold on;
        ylabel('MAE', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('MAE integral', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end
    if PARAMETERS.SIMULATION_INITIAL_POSITIONS  == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,4)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'r', 'LineWidth',1.0); 
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,4)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'b', 'LineWidth',1.0);    
    end
      
%     legend('Adaptive Sliding Mode (González-Prieto et al.)', 'Concise Backstepping (Zhang et al.)','Synergetic (Muhammad et al.)',...
%         'Interpreter','latex','FontSize', PARAMETERS.LEGEND_FONT_SIZE,'location','southeast'); 

    
    subplot(3,1,2);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,5)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'k', 'LineWidth',1.0);    
    
    if create
        grid on;
        hold on;
        ylabel('MIA', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('MIA integral', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end
    if  PARAMETERS.SIMULATION_TYPE == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,5)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'r', 'LineWidth',1.0);    
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,5)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'b', 'LineWidth',1.0);    
    end
    
    if PARAMETERS.SIMULATION_TYPE == 1
        axes('Position',[.55 .50 .3 .1])
        box on
        data_size = size(SIMULATION_DATA.data(:,1),1) - 1;
        detail_size = floor(20.0/PARAMETERS.SAMPLING_TIME);  
        from_to_detail = data_size-detail_size:data_size; 
        plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.AISM.data(from_to_detail,5)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'k',  'LineWidth',1.0);
        grid on;
        hold on;
        plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.NICB.data(from_to_detail,5)/PARAMETERS.TOTAL_TIME,'-', 'Color', 'r', 'LineWidth',1.0);
        plot(SIMULATION_DATA.data(from_to_detail,1), SIMULATION_DATA.SYN.data(from_to_detail,5)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'b', 'LineWidth',1.0); 
        xlim([PARAMETERS.TOTAL_TIME-detail_size*PARAMETERS.SAMPLING_TIME, PARAMETERS.TOTAL_TIME]);   
        MIA = 0.011348763656449;
        delta_mia = 1e-6;
        ylim([MIA-delta_mia,MIA+delta_mia]);     
        %ylim([0.011347,0.011350]);     
    end

    subplot(3,1,3);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,6)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'k', 'LineWidth',1.0);        
    if create
        grid on;
        hold on;
        ylabel('MTV', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('MTV integral', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0.0, PARAMETERS.TOTAL_TIME]);
    end
    if PARAMETERS.SIMULATION_TYPE == 1
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.NICB.data(:,6)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'r', 'LineWidth',1.0); 
        plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.SYN.data(:,6)/PARAMETERS.TOTAL_TIME ,'-', 'Color', 'b', 'LineWidth',1.0); 
    end
    fig4 = figure(4);
    if create
        clf(fig4);
    end
    subplot(4,1,1);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,9) ,'-', 'Color', 'k', 'LineWidth',1.0); 
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$\alpha(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Evolution of parameter $\alpha(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    subplot(4,1,2);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,11) ,'-', 'Color', 'k', 'LineWidth',1.0);   
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$\lambda(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Evolution of parameter $\lambda(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    subplot(4,1,3);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,12) ,'-', 'Color', 'k', 'LineWidth',1.0);   
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$\delta(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Evolution of parameter $\delta(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    subplot(4,1,4);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,13) ,'-', 'Color', 'k', 'LineWidth',1.0);   
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$\kappa(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Evolution of parameter $\kappa(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    fig5 = figure(5);
    if create
        clf(fig5);
    end
    subplot(2,1,1);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.AISM.data(:,10) ,'-', 'Color', 'k', 'LineWidth',1.0);   
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('s(t)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Adaptive sliding mode surface variable', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    subplot(2,1,2);
    plot(SIMULATION_DATA.data(:,1), SIMULATION_DATA.data(:,5) ,'-', 'Color', 'k', 'LineWidth',1.0);   
    if create
        grid on;
        hold on;
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$d(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Evolution of disturbance $d(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0, PARAMETERS.TOTAL_TIME]);
    end

    if PARAMETERS.CREATE_PDF
        if PARAMETERS.SIMULATION_TYPE == 1
            if PARAMETERS.DISTURBANCE_TYPE == 1
                figure(1);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_disturbance_states_control.pdf', '-transparent', '-nocrop');
                figure(2);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_disturbance_performance.pdf', '-transparent', '-nocrop');
                figure(4);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_disturbance_parameters.pdf', '-transparent', '-nocrop');
                figure(5);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_disturbance_s_d.pdf', '-transparent', '-nocrop');
            else                
                figure(1);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_states_control.pdf', '-transparent', '-nocrop');
                figure(2);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_performance.pdf', '-transparent', '-nocrop');
                figure(4);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_parameters.pdf', '-transparent', '-nocrop');
                figure(5);
                export_fig('../MANUSCRIPT/GRAPHICS/constant_reference_s_d.pdf', '-transparent', '-nocrop');
            end
        else
            if PARAMETERS.DISTURBANCE_TYPE == 1
                figure(1);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_disturbance_states_control.pdf', '-transparent', '-nocrop');
                figure(2);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_disturbance_performance.pdf', '-transparent', '-nocrop');
                figure(4);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_disturbance_parameters.pdf', '-transparent', '-nocrop');
                figure(5);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_disturbance_s_d.pdf', '-transparent', '-nocrop');
            else                
                figure(1);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_states_control.pdf', '-transparent', '-nocrop');
                figure(2);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_performance.pdf', '-transparent', '-nocrop');
                figure(4);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_parameters.pdf', '-transparent', '-nocrop');
                figure(5);
                export_fig('../MANUSCRIPT/GRAPHICS/sinusoidal_reference_s_d.pdf', '-transparent', '-nocrop');
            end
        end
    end    
end
