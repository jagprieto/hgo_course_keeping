clc;
clear('all');
rng('default');
warning('off','all');

%%% Practical proportional integral sliding mode control for underactuated %%%
%%% surface ships in the fields of marine practice %%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PARAMETERS = {};
PARAMETERS.CREATE_PDF = true;
PARAMETERS.SAMPLING_TIME = 1e-2;
PARAMETERS.TOTAL_TIME = 5;
PARAMETERS.MANEUVERABILITY_GAINS = [pi/8,pi/14,pi/20];
PARAMETERS.R_MAX = 0.3;
PARAMETERS.YAW_REF = 1.0;
PARAMETERS.BACKSTEPPING_GAIN = 2.0;
PARAMETERS.RUDDER_MAX = 0.91;
PARAMETERS.DOT_D_MAX = 2.0;
PARAMETERS.K = 0.336;
PARAMETERS.T = 0.24;
PARAMETERS.a1 = 0.35;
PARAMETERS.a2 = 0.3;
PARAMETERS.PLOT_FONT_SIZE = 12.0;
PARAMETERS.SIMULATION_TYPE = 2; % 1->Straight line; 2->Sinusoidal path
PARAMETERS.FULL_STATE_FEEDBACK = true; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate
if (PARAMETERS.FULL_STATE_FEEDBACK)
    run_full_states_simulation(PARAMETERS);
else
    run_partial_states_simulation(PARAMETERS);
end

function run_partial_states_simulation(PARAMETERS)
    if PARAMETERS.SIMULATION_TYPE  == 1
        PARAMETERS.initial_state = [0.00, 0.0]; % yaw, r
    else
        PARAMETERS.initial_state = [0.25, 0.1]; % yaw, r
    end
    PARAMETERS.OBSERBABILITY_TYPE = 1; % 1->yaw and r
    PARAMETERS.MANEUVERABILITY_GAIN = PARAMETERS.MANEUVERABILITY_GAINS(2);
    simulation_data_full = run_simulation(PARAMETERS);
    PARAMETERS.OBSERBABILITY_TYPE = 2; % 2-> only yaw
    simulation_data_partial = run_simulation(PARAMETERS);
    plot_simulation(simulation_data_full, 'r', true, PARAMETERS);
    plot_simulation(simulation_data_partial, 'b', false, PARAMETERS);
    figure(1);
    subplot(3,1,3);
    legend('Full state feedback','Partial state feddback', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
    if PARAMETERS.CREATE_PDF
        figure(1);
        if PARAMETERS.SIMULATION_TYPE  == 1
            export_fig('../MANUSCRIPT/GRAPHICS/partial_states_straight_line.pdf', '-transparent');
        else
            export_fig('../MANUSCRIPT/GRAPHICS/partial_states_curve.pdf', '-transparent');
        end
    end
end

function run_full_states_simulation(PARAMETERS)
    if PARAMETERS.SIMULATION_TYPE  == 1
        PARAMETERS.initial_state = [0.05, 0.0]; % yaw, r
    else
        PARAMETERS.initial_state = [0.25, 0.1]; % yaw, r
    end
    PARAMETERS.OBSERBABILITY_TYPE = 1; % 1->yaw and r; 2-> Only yaw
    PARAMETERS.MANEUVERABILITY_GAIN = PARAMETERS.MANEUVERABILITY_GAINS(1);
    simulation_data_1 = run_simulation(PARAMETERS);
    PARAMETERS.MANEUVERABILITY_GAIN = PARAMETERS.MANEUVERABILITY_GAINS(2);
    simulation_data_2 = run_simulation(PARAMETERS);
    PARAMETERS.MANEUVERABILITY_GAIN = PARAMETERS.MANEUVERABILITY_GAINS(3);
    simulation_data_3 = run_simulation(PARAMETERS);
    plot_simulation(simulation_data_1, 'r', true, PARAMETERS);
    plot_simulation(simulation_data_2, 'b', false, PARAMETERS);
    plot_simulation(simulation_data_3, 'k', false, PARAMETERS);
    figure(1);
    subplot(3,1,3);
    legend('$\mu={{\pi}\over{8}}$','$\mu={{\pi}\over{14}}$', '$\mu={{\pi}\over{20}}$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
    if PARAMETERS.CREATE_PDF
        figure(1);
        if PARAMETERS.SIMULATION_TYPE  == 1
            export_fig('../MANUSCRIPT/GRAPHICS/full_states_straight_line.pdf', '-transparent');
        else
            export_fig('../MANUSCRIPT/GRAPHICS/full_states_curve.pdf', '-transparent');
        end
        if PARAMETERS.SIMULATION_TYPE  == 1
            figure(2);
            export_fig('../MANUSCRIPT/GRAPHICS/full_states_disturbance_estimation.pdf', '-transparent');
        end
    end
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run simulation 
function simulation_data = run_simulation(PARAMETERS)
    % Simulation time
    simulation_time = 0:PARAMETERS.SAMPLING_TIME:PARAMETERS.TOTAL_TIME-PARAMETERS.SAMPLING_TIME;
    simulation_steps = size(simulation_time, 2);
    simulation_data = zeros(simulation_steps, 20);
    simulation_time = 0.0;
    
    % Inital states
    yaw = PARAMETERS.initial_state(1);
    r = PARAMETERS.initial_state(2);
    w_z_r = 0;
    if PARAMETERS.OBSERBABILITY_TYPE == 1 % yaw and r
        est_r = r;
    else
        est_r = 0;
    end
    beta_r = PARAMETERS.DOT_D_MAX;
    est_disturbance_r = 0;
    
    for simulation_step = 1:simulation_steps        
        %%%%%%%%%%%%%%%% Disturbance %%%%%%%%%%%%%%%%%% 
        disturbance = 0.3*cos(simulation_time);
        
        %%%%%%%%%%%%%%%% Reference %%%%%%%%%%%%%%%%%%%%  
        if PARAMETERS.SIMULATION_TYPE  == 1
            yaw_ref = PARAMETERS.YAW_REF;
            r_ref = 0;
            dot_r_ref = 0;
        else
            yaw_ref = 0.5*sin(0.2*simulation_time);
            r_ref = 0.2*0.5*cos(0.2*simulation_time);
            dot_r_ref = -0.2*0.2*0.5*sin(0.2*simulation_time);
        end
         
        %%%%%%%%%%%%%%%% Ship functions %%%%%%%%%%%%%%%%%%% 
        h_r = PARAMETERS.a1*r+PARAMETERS.a2*r^3; 
        g_r = (PARAMETERS.K/PARAMETERS.T);
        f_r = -g_r*h_r ;
        
        if PARAMETERS.OBSERBABILITY_TYPE == 2
            hat_h_r = PARAMETERS.a1*est_r+PARAMETERS.a2*est_r^3; 
            hat_f_r = -g_r*hat_h_r ;
        end
        
        %%%%%%%%%%%%%%%% Controller %%%%%%%%%%%%%%%%%%
        U_yaw = PARAMETERS.R_MAX
        PARAMETERS.SETTLING_TIME = 1 / U_yaw;
        alfa_yaw = U_yaw/PARAMETERS.MANEUVERABILITY_GAIN
        e_yaw = yaw - yaw_ref;
        dot_e_yaw = r - r_ref;
        r_c = r_ref - saturation(alfa_yaw*e_yaw,U_yaw);
        dot_r_c = dot_r_ref;
        if abs(alfa_yaw*e_yaw) < U_yaw
            dot_r_c = dot_r_c - alfa_yaw*dot_e_yaw;
        end
        U_r = PARAMETERS.BACKSTEPPING_GAIN*U_yaw     
        %alfa_r = (U_r*PARAMETERS.SETTLING_TIME)/PARAMETERS.MANEUVERABILITY_GAIN;
        nu_r = PARAMETERS.MANEUVERABILITY_GAIN/PARAMETERS.BACKSTEPPING_GAIN
        alfa_r = U_r/nu_r
       
        if PARAMETERS.OBSERBABILITY_TYPE == 1 % yaw and r
            e_r =  r - r_c;
            u = (1/g_r)*(-f_r - est_disturbance_r  + dot_r_c - saturation(alfa_r*e_r,U_r));
        else
            e_r =  est_r - r_c;
            u = (1/g_r)*(-hat_f_r - est_disturbance_r  + dot_r_c - saturation(alfa_r*e_r,U_r));
        end
        u = saturation(u,PARAMETERS.RUDDER_MAX); 
       
        %%%%%%%%%%%%%%%% Disturbance estimation %%%%%%%%%%%%%%%%%
        if PARAMETERS.OBSERBABILITY_TYPE == 1 % yaw and r
            [z_r_new, est_disturbance_r, est_r_new, w_z_r_new] = differenciation(r, f_r + g_r*u, est_r, w_z_r, PARAMETERS.SAMPLING_TIME, beta_r);
        else
            [z_r_new, est_disturbance_r, est_r_new, w_z_r_new] = differenciation(r, hat_f_r + g_r*u, est_r, w_z_r, PARAMETERS.SAMPLING_TIME, beta_r);
        end
        %%%%%%%%%%%%%%%%%%%%%% Ship dynamics %%%%%%%%%%%%%%%%%%%%%        
        dot_yaw = r;
        dot_r = f_r + g_r*u + disturbance;
        
        %%%%%%%%%%%%%%%%%%%%%% Save data %%%%%%%%%%%%%%%%%%%%%
        simulation_data(simulation_step, 1) = simulation_time;
        simulation_data(simulation_step, 2) = yaw_ref;
        simulation_data(simulation_step, 3) = r_ref;
        simulation_data(simulation_step, 4) = yaw;
        simulation_data(simulation_step, 5) = r; 
        simulation_data(simulation_step, 6) = u;
        if PARAMETERS.OBSERBABILITY_TYPE == 1
            simulation_data(simulation_step, 7) = disturbance;
        else
            simulation_data(simulation_step, 7) = f_r + disturbance;
        end
        simulation_data(simulation_step, 8) = est_disturbance_r;
        simulation_data(simulation_step, 9) = beta_r;
        simulation_data(simulation_step, 10) = est_r;         
        
        %%%%%%%%%%%%%%%%%%%%%% Update data %%%%%%%%%%%%%%%%%%%%% 
        yaw = yaw + dot_yaw*PARAMETERS.SAMPLING_TIME;
        r = r + dot_r*PARAMETERS.SAMPLING_TIME;
        est_r = est_r_new;
        w_z_r = w_z_r_new;
        simulation_time = simulation_time + PARAMETERS.SAMPLING_TIME;
    end
    PARAMETERS
end

% Differenciation function dx = f + d -> estimation of d
function [z, est_d, est_x_new, w_z_new] = differenciation(x, f, est_x, w_z, tau, beta)
    lambda = 1 / tau;
    nu = (((tau/pi)^2)*(2*beta))^(1/3);
    gamma = beta / nu;
    z = x - est_x;
    est_d = lambda*z + w_z;
    dot_est_x = f + est_d;
    dot_w_z = ((lambda^2)/4.0)*z + saturation(gamma*z, beta);
    est_x_new = est_x + dot_est_x*tau;
    w_z_new = w_z + dot_w_z*tau;
end

% Plot simulation 
function plot_simulation(simulation_data, color, create, PARAMETERS)
    fig1 = figure(1);
    if create
        clf(fig1);
    end
    subplot(3,1,1);
    plot(simulation_data(:,1), simulation_data(:,4) ,'-', 'Color', color, 'LineWidth',1.0);    
    if create
        grid on;
        hold on;
        plot(simulation_data(:,1), simulation_data(:,2) ,'-', 'Color', [0.5 0.5 0.5], 'LineWidth',1.5);
        ylabel('$\psi_{r}(t)$ vs $\psi$(t)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Desired and obtained yaw angle (rad)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
    end
    subplot(3,1,2);
    plot(simulation_data(:,1), simulation_data(:,5),'-', 'Color', color,  'LineWidth',1.0);
    if create
        grid on;
        hold on;
        plot(simulation_data(:,1), simulation_data(:,3) ,'-', 'Color', [0.5 0.5 0.5], 'LineWidth',1.5); 
        ylabel('$r_{r}(t)$ vs $r(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Desired and obtained yaw rate (rad/s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
    end
    subplot(3,1,3);
    plot(simulation_data(:,1), simulation_data(:,6),'-', 'Color', color, 'LineWidth',1.0);
    if create
        grid on;
        hold on;
        ylabel('u(t)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        title('Control effort', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
    end
    
    if create
        fig2 = figure(2);
        clf(fig2);
%         subplot(2,1,1);
        plot(simulation_data(:,1), simulation_data(:,8) ,'-', 'Color', color, 'LineWidth',1.0);   
        grid on;
        hold on;
        plot(simulation_data(:,1), simulation_data(:,7) ,'-', 'Color', [0.5 0.5 0.5], 'LineWidth',1.0); 
        title('Disturbance estimation', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        ylabel('$d(t)$ vs $\hat{d}(t)$', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlabel('time (s)', 'FontSize', PARAMETERS.PLOT_FONT_SIZE,'Interpreter','latex');
        xlim([0,2]);
    end
end

% Saturation function
function y = saturation(x, x_max)
    if abs(x)> x_max
        y=x_max*sign(x);
    else 
        y=x;
    end
end

