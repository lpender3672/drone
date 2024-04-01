

load 6ax_raw.mat
dt_values = raw(2:end, 1) - raw(1:end-1, 1);
mean_Fs = 1 / mean(dt_values);
mean_Fs

accx = raw(:, 2);
accx = accx - mean(accx);
accy = raw(:, 3);
accz = raw(:, 4);

eulx = raw(:, 5);
euly = raw(:, 6);
eulz = raw(:, 7);

load parsed_isolated_marble_data_az

%% opt_NBK_axis(accx, mean_Fs, "X");
%% opt_NBK_axis(accy, mean_Fs, "Y");
opt_NBK_axis(accz, mean_Fs, "Z");



%% setup to compute AV and ASD
function []=opt_NBK_axis(sampled_axis, sample_freq, axis_name)
    L = length(sampled_axis);        % Length of the data set.
    Fs = sample_freq;                 % IMU sampling rate = 100Hz
    T = 1/Fs;                 % time difference
    data.freq = sampled_axis;        % computing allan variance plot
    data.rate = Fs;
    N_tau = 50;             % number of tau values at which to optimize
    tau_min = 0.04;         % min value of tau range
    tau_max = 500;          % max value of tau range
    tau = logspace(log10(tau_min/T),log10(tau_max/T),N_tau)*T;
    % compute ASD
    sprintf('Computing ASD')
    [avar] = allan(data,tau);
    asd(1:N_tau,1) = avar.sig2;
    [m,n] = size(asd);
    if n>m
        asd = asd'; % ensure it is a column
    end
    av(1:N_tau,1) = (asd.^2);
    % Compute weighting matrix for cost function
    er  = avar.sig2err;                     % ASD standard deviation
    c = 4 * avar.sig2err .* avar.sig2err;   % Discussion near IEEE CSM eqn. (15)
    W = inv(diag(c));                       % convert to diagonal weight matrix
    sprintf('Finished computing ASD')
    
    % save 'ASD_Marble_slab.mat' tau asd av W er c

    T_B = 20;                % correlation time [sec]
    % mu_B = 1/T_B;          % time constant parameter [Hz]
    N = 3.3e-3;              % random walk parameter [m/sec^(3/2)]
    K = 1.4e-4;              % rate random walk parameter [m/sec^(5/2)]
    B = 4e-4;                % Bias instability parameter [m/sec^2]
    
    S_N = N^2;                                  % IEEE CSM eq. (22)
    S_K = K^2;                                  % IEEE CSM eq. (25)
    S_B = (2*(B^2)*log(2))/(pi*0.4365^2*T_B);   % IEEE CSM eq. (35)
    theta_o  = [S_B,T_B,S_N,S_K]';              % IEEE CSM eq. (63) with different order
    phi      = cmpt_basis(tau,theta_o(2));
    [av_o]   = cmpt_AV(theta_o,phi);
    [Cost_o] = cmpt_cost(av,av_o,W);
    
    %% Optimal parameter search
    %    Golden section search over theta(2). See Wikopedia for description.
    %    To find the minimum Cost on [a,b]
    %
    
    % range of T_B in [a,b] seconds
    a=10;
    [theta_a,Cost_a]=cmpt_theta(tau,av,a,W);
    b=200;
    [theta_b,Cost_b]=cmpt_theta(tau,av,b,W);
    mu =  (sqrt(5) + 1)/2;      % Golden ratio
    tol=  0.01;
    cnt = 0;
    while abs(b - a) > tol
        c = b - (b - a) / mu;
        d = a + (b - a) / mu;
        [theta_c,Cost_c]=cmpt_theta(tau,av,c,W);
        [theta_d,Cost_d]=cmpt_theta(tau,av,d,W);
        [a,c,d,b
        Cost_a, Cost_c, Cost_d, Cost_b] % display search process
        % implement golden section search
        if Cost_c < Cost_d
            b = d;
            Cost_b = Cost_d;
            plot_AV_mdl_components(tau,av,theta_c,er)
        else
            a = c;
            Cost_a = Cost_c
            plot_AV_mdl_components(tau,av,theta_d,er)
        end
        pause_time = 1.0;   % seconds
        pause(pause_time)
    end
    [a,b
     Cost_a, Cost_b] % display search process
    theta_v = theta_c;                  % computed optimal value
    phi      = cmpt_basis(tau,theta_v(2));
    [av_v]=cmpt_AV(theta_v,phi);        % optimized fit to AV
    
    
    AV_SBSNSK = [theta_v([1,3,4]),theta_o([1,3,4])]              % display result
    ASD_NK = sqrt([theta_v([3,4]),theta_o([3,4])])              % display result
    TB = [theta_v(2),theta_o(2)]
    
    plot_AV_error(tau,av,av_v,av_o,W)
    save(append("results", axis_name , "_opt_NBK_parameters"), "AV_SBSNSK", "ASD_NK", "TB")
end


function []=plot_AV_error(tau,av_t,av_c,av_o,W)
% plot the AV and errors
% av_t --- computed from data
% av_c --- optimal fit by computation
% av_o --- from paper
err =  (av_t - av_c);   % curve fit error raw
figure(3)
clf
subplot(311)
loglog(tau,av_t,'b-','LineWidth',2)
grid on
hold on
loglog(tau,av_c,'r--',tau,av_o,'g--','LineWidth',2)
xlabel('cluster time (sec)','FontSize',14)
ylabel('AVAR','FontSize',14)
%ylim([1e-8 1e-2])
legend('Data','Optimized Mdl','Paper Mdl')  % Optimize based n (min AVAR sq err)

subplot(312)
loglog(tau,abs(err),'r.',tau,abs(av_t-av_o),'g.','LineWidth',2)
grid on
xlabel('cluster time (sec)','FontSize',14)
ylabel('AVAR raw Error','FontSize',14)
legend('curve fit','paper')


subplot(313)
loglog(tau,W*abs(err),'r.',tau,W*abs(av_t-av_o),'g.','LineWidth',2)
grid on
xlabel('cluster time (sec)','FontSize',14)
ylabel('AVAR weighted Error','FontSize',14)
legend('curve fit','paper')
end

function []=plot_AV_mdl_components(tau,av,theta,err)
TB = theta(2);
[phi]=cmpt_basis(tau,TB);
[av_T]=cmpt_AV(theta,phi);
av_B = phi(:,1)*theta(1);
av_N = phi(:,3)*theta(3);
av_K = phi(:,4)*theta(4);
figure(4); clf
loglog(tau,av,'.')
xlabel('Delay, \tau, s')
ylabel('AV')
grid
ylim([1e-8 1e-4])
hold on
loglog(tau,[av_N,av_B,av_K,av_T])
hold off
legend('AV actual','AV_N','AV_B','AB_K','AV-fit')
title('AV optimization-based fitting process')

figure(5)
loglog(tau,sqrt(av),'.')
xlabel('Delay, \tau, s')
ylabel('ASD')
grid
ylim([1e-4 1e-2])
hold on
loglog(tau,sqrt([av_N,av_B,av_K,av_T]))
hold off
legend('ASD actual','ASD_N','ASD_B','ASD_K','ASD-fit')
title('ASD optimization-based fitting process')

figure(15)
errorbar(tau,sqrt(av),err);
ylim([1e-4 1e-2])
set(gca,'XScale','log','YScale','log');
xlabel('Delay, \tau, s')
ylabel('ASD')
grid
hold on
loglog(tau,sqrt([av_N,av_B,av_K,av_T]))
hold off
legend('ASD actual','ASD_N','ASD_B','ASD_K','ASD-fit')
title('ASD optimization-based fitting process')

end

function [Cost]=cmpt_cost(av_t,av_c,W)
% compute the cost functions that we are trying to minimize
Cost =  (av_t - av_c)'*W*W*(av_t - av_c);
end

function [phi]=cmpt_basis(tau,theta_2)
% follows notation preceeding IEEE CSM eqn (63) in a different order:
%   theta(1) is IEEE CSM theta_2
%   theta(2) is IEEE CSM theta_4 (nonlinear in IEEE CSM eqn. (63))
%   theta(3) is IEEE CSM theta_1 
%   theta(4) is IEEE CSM theta_3 
% phi(:,2) is not needed here because theta(2) is not linear

a(:,1) = (theta_2./tau)';       % worker variable
b      = ones(size(a));         % worker variable
c      = b./a;                  % worker variable
phi(:,1) = theta_2*a.*(b-(a/2).*(3*b-4*exp(-c)+exp(-2*c)));
phi(:,3) = 1./tau;      % coefficeint of theta(3)
phi(:,4) = tau./3;      % coefficient of theta(4)
end

function [theta,Cost]=cmpt_theta(tau,av,theta_2,W)
% Given a value for theta(2), optimize the other parameters that appear linearly

[phi]      = cmpt_basis(tau,theta_2);
H          = phi(:,[1,3,4]);                % discard column 2
lambda     = inv(H'*W*H)*H'*W*av;           % LS over theta 1, theta 3, theta 4
theta      = [lambda(1);theta_2;lambda(2);lambda(3)];
[avc]      = cmpt_AV(theta,phi);
[Cost]     = cmpt_cost(av,avc,W);
end

function [av]=cmpt_AV(theta,phi)
% follows notation preceeding IEEE CSM eqn (63) in a different order:
%   theta(1) is IEEE CSM theta_2
%   theta(2) is IEEE CSM theta_4 (nonlinear in IEEE CSM eqn. (63))
%   theta(3) is IEEE CSM theta_1 
%   theta(4) is IEEE CSM theta_3 
% phi(:,2) is zero
% theta(2) is accounted for in phi(1)
lambda    = [theta(1);theta(3);theta(4)];
av(:,1)  = phi(:,[1,3,4])*lambda;
end
