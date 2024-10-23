%
% RLE SSSHA Assessment Script
%
% Sep 2024
%

%% Script flags
rle_id = 1;
reload_model = true; %false; %
plot_sssha_acc = true;
plot_hpf = false;
compare_sa = false;

% dt_file = './data/model-RLE01_wiM1c_noSGMC_1.parquet';
% dt_file = './data/model-20241003_1800-25Hzmodes-RLE01_wiM1c_wiSGMC_1.parquet'; fig_offset = 60;
% dt_file = sprintf('./data/model-20241003_1800-RLE0%d_woM1c_wiSGMC_1.parquet',rle_id); fig_offset = 40;
% dt_file = sprintf('./data/model-20240408_1535-RLE0%d_wiM1c_wiSGMC_1.parquet',rle_id); fig_offset = 0+rle_id*1000;
% dt_file = sprintf('./data/model-20230817_1808-RLE0%d_wiM1c_wiSGMC_1.parquet',rle_id); fig_offset = 100+rle_id*1000;
dt_file = sprintf('./data/model-20241021_1535-RLE0%d_wiM1c_wiSGMC_1.parquet',rle_id); fig_offset = 120+rle_id*1000;
% dt_file = sprintf('./data/model-20230817_1808-95Hzmodes-RLE0%d_wiM1c_wiSGMC_1.parquet',rle_id); fig_offset = 80;


dt_file_label = replace(extractBetween(dt_file,"data/","_1.parquet"),'_','\_');

% PSD settings (Welch algorithm)
psd.M = 1024;
psd.nFFT = 2^14;
psd.detrend = 'none';

%% Simulation data
%%
try
    parquetINFO = parquetinfo(dt_file);
    sssha_data = parquetread(dt_file,"SampleRate",1e3,...
        "SelectedVariableNames",parquetINFO.VariableNames);
    % "OSSPayloads6D";"OSS00GroundAcc";"OSSHardpointD";"OSSM1Lcl";"MountEncoders"
catch
    warning('Unable to run parquetread(). Try Matlab 2022b, or later.');
end

t = seconds(sssha_data.Time);
Ts = diff(t(1:2));
HP_D = reshape(cell2mat(sssha_data.OSSHardpointD),84,[]);
mnt_enc = reshape(cell2mat(sssha_data.MountEncoders),14,[]);


%% Acceleration and Mount ENC plots
%%
t_range = [0.005,max(t)];%[];%[4.19,4.208];%[];%
if(isempty(t_range))
    t_idx  = [1, length(t)];
else
    t_idx = [find(t >= t_range(1),1,"first"),find(t < t_range(2),1,"last")];
end

if (any(contains(parquetINFO.VariableNames,"OSS00GroundAcc")) && 1)
    gnd_acc = reshape(cell2mat(sssha_data.OSS00GroundAcc),3,[]);
    
    % GND_ACC plot
    if(plot_sssha_acc)
        figure(998)
        set(gcf,'position',[123   230   400   400])
        subplot(2,1,1)
        plot(t(t_idx(1):t_idx(2)),gnd_acc(:, t_idx(1):t_idx(2))');
        set(gca,'ColorOrderIndex',1); hold on;
        plot(t([t_idx(1),t_idx(2)]), kron([1 1],mean(gnd_acc,2)), '--');
        ylabel('Accelerations (m/s^2)');
        xlabel('Time (s)');
        legend('H1','H2','V');
        title(dt_file_label)
        grid on; axis tight; hold off

        acc_ps = zeros(psd.nFFT/2+1, 3);
        for ik = 1:3
            [acc_ps(:,ik),freqP] = utils.pwelch(gnd_acc(ik,:)'...
                ,2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
        end
        subplot(2,1,2)
        semilogx(freqP, acc_ps);
        xlabel('Frequency (Hz)');
        ylabel('GND Acc ((m/s^2)^2/Hz)');
        grid on; axis tight;
    end

    gnd_num_x = cumsum(Ts*cumsum(Ts*gnd_acc(:, t_idx(1):t_idx(2))'));
    if (any(contains(parquetINFO.VariableNames,"OSS00Ground6D")) && 1)        
        gnd_D = reshape(cell2mat(sssha_data.OSS00Ground6D),6,[]);
        account4lmk = false;
        if(account4lmk)
            gnd_num_x = gnd_num_x - 8.5294e9/2.1605e12*...
                cumsum(Ts*cumsum(Ts*gnd_D(1:3, t_idx(1):t_idx(2))'));
            warning("Accounting for the large mass spring on the GND position calculation!");
        end
    else
        gnd_D = [];
    end

    % GND motion verification plot
    figure(898+rle_id*1000)
    set(gcf,'position',[423   250   740   400])
    plot(t(t_idx(1):t_idx(2)),gnd_num_x, '-');
    hold on;
    legend_str = {'\int\int H1 acc','\int\int H2 acc','\int\int V acc'};
        
    if(~isempty(gnd_D))
        set(gca,'ColorOrderIndex',1); 
        plot(t(t_idx(1):t_idx(2)), gnd_D(1:3, t_idx(1):t_idx(2))',':','Linewidth',2);
        legend_str = [legend_str(:)',{'GND x motion'},{'GND y motion'},{'GND z motion'}];
%         plot(t(t_idx(1):t_idx(2)), gnd_D(4:6, t_idx(1):t_idx(2))', '-'); % ZERO        
    end
    if(any(contains(parquetINFO.VariableNames,"Pier6D")) && 1)
        pier_D = reshape(cell2mat(sssha_data.Pier6D),12,[]);
        plot(t(t_idx(1):t_idx(2)), pier_D(1:3, t_idx(1):t_idx(2))', '-.');
        legend_str = [legend_str(:)',{'Pier(B) X^{\rightarrow}'},...
            {'Pier(B) Y^{\rightarrow}'},{'Pier(B) Z^{\rightarrow}'}];
        set(gca,'ColorOrderIndex',4);
        plot(t(t_idx(1):t_idx(2)), pier_D((1:3)+6, t_idx(1):t_idx(2))',...
            '--','Linewidth',1.5);
        legend_str = [legend_str(:)',{'Pier(T) X^{\rightarrow}'},...
            {'Pier(T) Y^{\rightarrow}'},{'Pier(T) Z^{\rightarrow}'}];
    end
    
    ylabel('Motion (m)');
    xlabel('Time (s)');
    legend(legend_str,'Location','Southeast');
    title(dt_file_label)
    grid on; axis tight; hold off;
end

if(exist('mnt_enc','var'))
    figure(997)
    set(gcf,'position',[323   130   400   400])
    az_enc = mean(mnt_enc(1:4,t_idx(1):t_idx(2)))';
    el_enc = mean(mnt_enc(5:10,t_idx(1):t_idx(2)))';
    gir_enc = mean(mnt_enc(11:14, t_idx(1):t_idx(2)))';
    subplot(2,1,1)
%     yyaxis left;
    plot(t(t_idx(1):t_idx(2)),[az_enc, el_enc, gir_enc]);
    ylabel('MNT ENC (rad)');
    xlabel('Time (s)');
    legend('AZ','EL','GIR');
    title(dt_file_label)
    grid on; axis tight;
    rad_lim = get(gca,'YLim');
    yyaxis right;
    ylim(180/pi*3600*rad_lim);
    ylabel('MNT ENC (arc sec)');
    

    mnt_ps = zeros(psd.nFFT/2+1, 3);
    [mnt_ps(:,1),freqP] = utils.pwelch(az_enc,...
        2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
    [mnt_ps(:,2),~] = utils.pwelch(el_enc,...
        2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
    [mnt_ps(:,3),~] = utils.pwelch(gir_enc,...
        2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
    subplot(2,1,2)
    semilogx(freqP, mnt_ps);
    xlabel('Frequency (Hz)'); ylabel('Mount Enc (rad^2/Hz)'); grid on; axis tight;
    rad2_lim = get(gca,'YLim');
    yyaxis right;
    ylim((180/pi*3600)^2*rad2_lim);
    ylabel('MNT ENC (asec^2/Hz)');
end


%% Pier ACC & Motion plots
%%
if(any(contains(parquetINFO.VariableNames,"Pier6D")) && 1)
    pier_D = reshape(cell2mat(sssha_data.Pier6D),12,[]);

    plot_ddot = true;%false;
    pier_sel = 1:6;
    pier_b_x = pier_D(pier_sel,t_idx(1):t_idx(2))';    
    pier_b_xddot = 1/Ts*diff(1/Ts*diff(pier_D(pier_sel,t_idx(1)-2:t_idx(2))'));

    if(compare_sa)                
%         dt_file__ = './data/model-20230817_1808-95Hzmodes-RLE01_wiM1c_wiSGMC_1.parquet';
%         dt_file__ = './data/model-20230817_1808-RLE01_wiM1c_wiSGMC_1.parquet';
        dt_file__ = spriintf('./data/model-20230817_1808-RLE0%d_wiM1c_wiSGMC_1.parquet',...
            rle_id);
        sssha_data = parquetread(dt_file__,"SampleRate",1e3,...
            "SelectedVariableNames","Pier6D");
        pier_D_ = reshape(cell2mat(sssha_data.Pier6D),12,[]);
        pier_acc_ks_nom = 1/Ts*diff(1/Ts*diff(pier_D_(1:3,t_idx(1)-2:t_idx(2))'));
    end
    
%     pier_t = pier_D(7:12,t_idx(1):t_idx(2))';

    if(plot_ddot)
        fid_offset = 10;%2;
        pier_out_dt = pier_b_xddot;
        y_label1 = 'Pier Acc time response (%s/s^2)';
        y_label2 = 'Pier Acc PSD (%s^2/s^4/Hz)';
        plot_title_str = "Bottom Pier Node Acceleration";
%         pier_out_dt = 1/Ts*diff(1/Ts*diff(...
%             pier_D(1:6,t_idx(1)-2:t_idx(2))' - gnd_D(:, t_idx(1)-2:t_idx(2))'));
%         y_label1 = 'Relative Acc time response (%s/s^2)';
%         y_label2 = 'Relative Acc PSD (%s^2/s^4/Hz)';
%         plot_title_str = "Pier(B)-GND Relative Acceleration";
    else
        fid_offset = 8;%4;%0;
%         pier_out_dt = pier_b_x;
%         y_label1 = 'Pier motion time response (%s)';
%         y_label2 = 'Pier motion PSD (%s^2/Hz)';

%         y_label1 = '(T-B) motion time response (%s)';
%         y_label2 = '(T-B) motion PSD (%s^2/Hz)';
%         plot_title_str = "Relative pier motion (Top-bottom)";
%         pier_out_dt = pier_D(7:12,t_idx(1):t_idx(2))' - pier_D(1:6,t_idx(1):t_idx(2))';

        y_label1 = '(B-GND) motion time response (%s)';
        y_label2 = '(B-GND) motion PSD (%s^2/Hz)';
        plot_title_str = "Relative bottom pier motion (Bottom Pier - GND)";
        pier_out_dt = pier_D(1:6,t_idx(1):t_idx(2))' - gnd_D(:, t_idx(1):t_idx(2))';
    end
    legend_str = {'X^{\rightarrow}','Y^{\rightarrow}','Z^{\rightarrow}'};

    pier_b_Txyz_ps = zeros(psd.nFFT/2+1, 3);
    pier_b_Rxyz_ps = zeros(psd.nFFT/2+1, 3);
    for iu=1:3
        [pier_b_Txyz_ps(:,iu),freqP] = utils.pwelch(pier_out_dt(:,iu),...
            2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
        [pier_b_Rxyz_ps(:,iu),~] = utils.pwelch(pier_out_dt(:,iu+3),...
            2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
    end

    
    figure(996-fid_offset)
    set(gcf,'position',[123   80   740   400])
    subplot(2,1,1)
    plot(t(t_idx(1):t_idx(2)),pier_out_dt(:,1:3));
    ylabel(sprintf(y_label1,'m'));
    xlabel('Time (s)'); grid on; axis tight;
    legend(legend_str);    
    title(plot_title_str)
    subplot(2,1,2)
    semilogx(freqP, pier_b_Txyz_ps);
    xlabel('Frequency (Hz)'); ylabel(sprintf(y_label2,'m')); grid on; axis tight;

% ROTATIONS    
%     figure(995-fid_offset)
%     set(gcf,'position',[423   150   740   400])
%     subplot(2,1,1)
%     plot(t(t_idx(1):t_idx(2)),pier_out_dt(:,4:6));
%     ylabel(sprintf(y_label1,'rad'));
%     xlabel('Time (s)'); grid on; axis tight;
%     legend(legend_str);
%     title(plot_title_str)
%     subplot(2,1,2)
%     semilogx(freqP, pier_b_Rxyz_ps);
%     xlabel('Frequency (Hz)'); ylabel(sprintf(y_label2,'rad')); grid on; axis tight;
end

%% RLE Spectral Acceleration
%%

pier_acc_dt = 1/Ts*diff(1/Ts*diff(...
            pier_D(1:3,t_idx(1)-2:t_idx(2))' - 0*gnd_D(1:3, t_idx(1)-2:t_idx(2))'));
pier_Racc_dt = 1/Ts*diff(1/Ts*diff(...
            pier_D(1:3,t_idx(1)-2:t_idx(2))' - gnd_D(1:3, t_idx(1)-2:t_idx(2))'));


% [SRS_STRUCT] = compute_response_spectra (EQ_STRUCT, ZETA)
% B. Smith, 18 Sept 2024
zeta = .02; % payload damping ratio
STEPS = 4; % number of frequency steps per FWHM, 4 yields <3.6% scalloping
FMIN = 1; %  minimum frequency [Hz]

fs = 1/Ts; % sampling frequency
dT = 1/fs;
fmax = fs/2;
q = 1/(2*zeta); % resonance Q factor, Q = f/FWHM

nFreqs = round(log(fmax/FMIN)/log(1+1/(q*STEPS)));
fSRS = logspace(log10(FMIN),log10(fmax),nFreqs); % frequency vector [Hz]

sa_data = zeros(nFreqs, min(size(pier_acc_dt))+1); % preallocate result
sRa_data = zeros(size(sa_data));
if(compare_sa), sa_ksnom_data = zeros(nFreqs, min(size(pier_acc_ks_nom))+1); end
sa_data(:,1) = fSRS;
for j = 2:size(sa_data,2)
    sa_data(:,j) = SpectralA04(pier_acc_dt(:,j-1), fSRS, dT, zeta);
    sRa_data(:,j) = SpectralA04(pier_Racc_dt(:,j-1), fSRS, dT, zeta);
    if(compare_sa)
        sa_ksnom_data(:,j) = SpectralA04(pier_acc_ks_nom(:,j-1), fSRS, dT, zeta);
    end
end

%%
figure(910-fid_offset)
set(gcf,'position',[423   150   740   400])
ylabel_str = ["H1","H2","V"];
for ik = 1:3    
    subplot(3,1,ik)
    plot(sa_data(:,1),sa_data(:,ik+1))
    hold on;
%     plot(sa_data(:,1),sRa_data(:,ik+1))
    if(compare_sa), plot(sa_data(:,1),sa_ksnom_data(:,ik+1),'--'); end
    xlim([0, 100])
    grid on; ylabel(ylabel_str{ik}+" (m/s^2)"); hold off;
end
xlabel("Frequency (Hz)")
subplot(3,1,1);
title(sprintf("RLE%d - Spectral acceleration response", rle_id));
% legend('Pier (Bottom)','Rel Acc Pier(B)-GND')
if(compare_sa), legend('Reduced SI lateral sitffness','Nominal stiffness'); end;

%%
if(~plot_hpf), return; end

%% Structural model data 
%% 

model_label = extractBetween(dt_file,"data/model-","-");

% ModelFolder = "20230817_1808_zen_30_M1_202110_FSM_202305_Mount_202305_IDOM_concreteAndFoundation_largeMass";
ModelFolder = "20240408_1535_zen_30_M1_202110_FSM_202305_Mount_202305_IDOM_concreteAndFoundation_finalSI_largeMass";
% ModelFolder = "20241003_1800_zen_30_M1_202110_FSM_202305_Mount_202305_HP202409_largeMass";
FileName = "modal_state_space_model_2ndOrder.mat";

assert(contains(ModelFolder, model_label));

if(~exist('inputTable','var') || reload_model)    
    load(fullfile(im.lfFolder,ModelFolder,FileName),...
        'inputs2ModalF','modalDisp2Outputs',...
        'eigenfrequencies','inputTable','outputTable');

    fprintf('Model %s loaded from\n%s\n', FileName, ModelFolder);
    fprintf("The model maximum eigenfrequency is %.5fHz.\n", eigenfrequencies(end));
    % Static solution gain matrix
    staticSolFile = fullfile(im.lfFolder,ModelFolder,"static_reduction_model.mat");
    try
        load(staticSolFile,'gainMatrixMountControlled');
        gainMatrix = gainMatrixMountControlled;
    catch
        load(staticSolFile,'gainMatrix');
    end

    fprintf('Static gain matrix loaded from\n%s\n', staticSolFile);
else
    fprintf('Using pre-loaded files\n');
end

% HP I/O indices
hp_i_in = inputTable{"OSS_Harpoint_delta_F","indices"}{1};
hp_i_out = outputTable{"OSS_Hardpoint_D","indices"}{1};
% MNT enc output indices
mntAZ_out = outputTable{'OSS_AzEncoder_Angle',"indices"}{1};
mntEL_out = outputTable{'OSS_ElEncoder_Angle',"indices"}{1};
mntGIR_out = outputTable{'OSS_RotEncoder_Angle',"indices"}{1};

Phi_mnt_pinv = pinv([mean(modalDisp2Outputs(mntAZ_out,1:3),1);...
    mean(modalDisp2Outputs(mntEL_out,1:3),1);...
    mean(modalDisp2Outputs(mntGIR_out,1:3),1)]);

Hkin_hp = modalDisp2Outputs(hp_i_out,1:3) * Phi_mnt_pinv;


%% HP stiffness
%%
T_hp = kron(eye(7),[-eye(6),eye(6)]);

HPk = 1./ diag(T_hp*gainMatrix(hp_i_out,hp_i_in));
fprintf('HP stiffness: (avg) %g (N/um), std:%g\n', mean(1e-6*HPk), std(1e-6*HPk))
if(exist('eigenfrequencies','var'))
    invOM2 = diag(1./((2*pi*eigenfrequencies).^2));
    HPk_modal = 1./diag(T_hp * modalDisp2Outputs(hp_i_out,:) * invOM2 * inputs2ModalF(:,hp_i_in));
    fprintf('HP stiffness (from modal model): (avg) %g (N/um), std:%g\n', mean(1e-6*HPk_modal), std(1e-6*HPk_modal))
end

% HP stiffness due to mode truncation
if false
    freq_lim_vec = (45:1:max(eigenfrequencies))'; %#ok<*UNRCH> 
    HPk_dyn = zeros(numel(freq_lim_vec),2);
    for iw = 1:numel(freq_lim_vec)
        maxf_i = find(eigenfrequencies <= freq_lim_vec(iw),1,"last");
        invOM2_ = diag(1./((2*pi*eigenfrequencies(1:maxf_i)).^2));
        HPk_ = 1./diag(T_hp * modalDisp2Outputs(hp_i_out,1:maxf_i) * invOM2_ * inputs2ModalF(1:maxf_i,hp_i_in));
        HPk_dyn(iw,:) = [mean(HPk_), std(HPk_)];
    end
    figure(999)    
    plot(freq_lim_vec,HPk_dyn(:,1),'s:');
    hold on; semilogy(max(eigenfrequencies),mean(HPk_modal),'x');
    grid on;
    xlabel('Max mode frequency (Hz)');
    ylabel('HP stiffness (N/m)')
end


%% Calculation of HP axial forces
%%
HPk_ = mean(HPk); %mean(HPk_modal); %
HP_axialF = HPk_ * T_hp * HP_D;
hp_sel = 1:42;
HP_raxialD = T_hp * HP_D;

% M1 HP breakaway force limits
f_compression = 1200;
f_tension = 900;


%% HP Force DEBUG PLOTs
%%
% HP_Fo_pointing = HPk_ * T_hp * Hkin_hp * [az_enc, el_enc, gir_enc]';
% Delta_HP_axialF = HPk_ * T_hp * (HP_D(:,t_idx(1):t_idx(2))...
%     - Hkin_hp * [az_enc, el_enc, gir_enc]');
% 
% figure(fig_offset+2)
% set(gcf,'position',[823   230   400   400])
% tk = t(t_idx(1):t_idx(2));
% plot(tk, HP_Fo_pointing(hp_sel,:)');
% ylabel('HP Forces induced by telescope pointing (N)')
% 
% grid on; hold off;
% % title(sprintf('HPk=%3.1fN/um', mean(HPk_)/1e6));
% xlabel('Time (s)'); axis tight;
% 
% figure(fig_offset+3)
% set(gcf,'position',[323   80   700   400])
% 
% plot(tk, Delta_HP_axialF(:,:)');
% ylabel('HP Forces without telescope pointing contribution (N)')
% 
% % [max_HPf, i_max] = max(abs(HP_axialF(:,t_idx(1):t_idx(2))));
% % [max_DHPf, imaxD] = max(abs(Delta_HP_axialF(:,t_idx(1):t_idx(2))));
% % plot(tk, max_HPf','s', tk, max_DHPf','.--');
% 
% hold on; plot(tk, f_compression*ones(size(tk)),'k--');
% plot(tk, -f_tension*ones(size(tk)),'k--'); 
% grid on; hold off;
% xlabel('Time (s)'); axis tight;


figure(fig_offset+2)
set(gcf,'position',[523   230   400   400])
tk = t(t_idx(1):t_idx(2));
plot(tk, HP_raxialD(hp_sel,t_idx(1):t_idx(2))');
ylabel('HP axial displacement')

grid on; hold off;
xlabel('Time (s)'); axis tight;
SE = [max(xlim) min(ylim)]-[diff(xlim) -diff(ylim)]*0.05;
text(SE(1), SE(2), dt_file_label,'Fontsize',12,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','right')



%% Plot of HP axial forces
%%

figure(fig_offset+1)
set(gcf,'position',[523   230   400   400])
tk = t(t_idx(1):t_idx(2));
plot(tk, HP_axialF(hp_sel,t_idx(1):t_idx(2))');
hold on; plot(tk, f_compression*ones(size(tk)),'k--');
plot(tk, -f_tension*ones(size(tk)),'k--'); 
grid on; hold off;
text(round(0.98*max(tk)), f_compression,...
    sprintf('Breakaway compression limit (%dN)',f_compression),...
    'VerticalAlignment','bottom', 'HorizontalAlignment','right');
text(round(0.98*max(tk)), -f_tension,...
    sprintf('Breakaway tension limit (%dN)',f_tension),...
    'VerticalAlignment','top', 'HorizontalAlignment','right');
ylabel(sprintf('HP Forces (N) HP stiffness=%3.1fN/um', mean(HPk_)/1e6));
%ylabel('HP axial displacement (m)');
xlabel('Time (s)'); axis tight;
% grid on; axis tight;
SE = [max(xlim) min(ylim)]-[diff(xlim) -diff(ylim)]*0.05;
text(SE(1), SE(2), dt_file_label,'Fontsize',12,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','right')

if (any(contains(parquetINFO.VariableNames,"OSSHardpointForce")) && 1)
    HPf_fem = reshape(cell2mat(sssha_data.OSSHardpointForce),42,[]);
    figure(fig_offset+4)
    set(gcf,'position',[523   130   400   400])
    tk = t(t_idx(1):t_idx(2));
    plot(tk, HPf_fem(hp_sel,t_idx(1):t_idx(2))');
    hold on; plot(tk, f_compression*ones(size(tk)),'k--');
    plot(tk, -f_tension*ones(size(tk)),'k--');
    grid on; hold off;
    text(round(0.98*max(tk)), f_compression,...
        sprintf('Breakaway compression limit (%dN)',f_compression),...
        'VerticalAlignment','bottom', 'HorizontalAlignment','right');
    text(round(0.98*max(tk)), -f_tension,...
        sprintf('Breakaway tension limit (%dN)',f_tension),...
        'VerticalAlignment','top', 'HorizontalAlignment','right');
    ylabel(sprintf('HP Forces* (N) - HP stiffness=%3.1fN/um', mean(HPk_)/1e6));    
    xlabel('Time (s)'); axis tight;
    % grid on; axis tight;
    SE = [max(xlim) min(ylim)]-[diff(xlim) -diff(ylim)]*0.05;
    text(SE(1), SE(2), dt_file_label,'Fontsize',12,...
        'VerticalAlignment','bottom', 'HorizontalAlignment','right')
end

%% HP Force PSD plot
%%
HP_f_ps = zeros(psd.nFFT/2+1,42);
for ik = 1:42
    [HP_f_ps(:,ik),freqP] = utils.pwelch(HP_axialF(ik,:)'...
        ,2*psd.M,[],psd.nFFT,1/Ts,'onesided',psd.detrend);
end

figure(6+fig_offset)
set(gcf,'position',[623   130   400   400])
subplot(2,1,1)
loglog(freqP, HP_f_ps(:,:));
title(dt_file_label)
xlabel('Frequency (Hz)'); ylabel('HP Forces PSD (N^2/Hz)'); grid on; axis tight;
subplot(2,1,2)
semilogx(freqP, sqrt(cumsum([0;diff(freqP)] .* HP_f_ps)));
xlabel('Frequency (Hz)'); ylabel('Cumulative PSD (N)'); grid on; axis tight;


%%
return








%% Pier TF
%%

in_pier_idx = inputTable{'Pier_6F','indices'}{1}(1:6)'; % 1:6 Bottom node F
out_pier_idx = outputTable{'Pier_6D','indices'}{1}(7:12)';

phiB_ = inputs2ModalF(:,in_pier_idx);
n_u = size(phiB_,2);
phiC_ = modalDisp2Outputs(out_pier_idx,:);
n_y = size(phiC_,1);

om0 = eigenfrequencies(:)*2*pi;
% Set structural damping
damp = 0.02;
damp_vec = damp*ones(size(om0));
% Number of frequency response points
Nom = 2000;
% Frequency range for analysis
wrange = [0.1,100];      
% Frequency points (rad/s)
omega = logspace(log10(wrange(1)),log10(wrange(2)),Nom)'*2*pi;

if(~exist('G_jw','var') || reload_model)
    % Compute frequency responses
    fprintf('Computing G(jw) with %d modes, zeta= %g, and w=2*pi*[%.3g ~ %.3g].\n',...
        length(om0), damp, min(omega)/2/pi ,max(omega)/2/pi);
    G_jw = frd(...
        freqresp_2ndorder_model(...
        phiB_,phiC_,omega,om0,damp_vec,1:n_u,1:n_y), omega);
end

figure(200)
set(gcf,'position',[523   130   400   400])
subplot(2,1,1)
loglog(omega/2/pi, 1e9*squeeze(abs(G_jw.ResponseData(1,1,:))),'-');
grid on; axis tight; hold on;
loglog(omega/2/pi, 1e9*squeeze(abs(G_jw.ResponseData(2,2,:))),'-');
loglog(omega/2/pi, 1e9*squeeze(abs(G_jw.ResponseData(2,1,:))),'--')
ylabel('Horizontal responses (nm/N)')
legend("F_x -> T_x", "F_y -> T_y", "Coupled response",...
    'Location','Southeast');
hold off;
subplot(2,1,2)
loglog(omega/2/pi, 1e9*squeeze(abs(G_jw.ResponseData(3,1:3,:))))
xlabel('Frequency (Hz)')
ylabel('Vertical response (nm/N)')
grid on; axis tight;

%% Spectral Acceleration response
%%
function [Sa] = SpectralA04(InputSignal,f,dt,d)
%   SpectralA04(InputSignal,f,dt,d)
%   delivers a vector of the spectral acceleration response of the signals
%   represented in the InputSignal variable.
%   f, corresponds to the frequency vector at which the Spectral [Hz]
%   acceleration is evaluated.
%   dt corresponds to the time step of the input signal [s]
%   d corresponds to the damping ratio at which the spectral response need
%   to be evaluated
%
% AO4 - updated to max abs
    ns=min(size(InputSignal)); % number of signals
    Sa=zeros(length(f),ns); % preallocate result variable
    for i1=1:length(f)

      % continuous transfer function for harmonic oscillator
      filt1=tf([f(i1)*4*pi*d (f(i1)*2*pi)^2],[1 d*4*f(i1)*pi (f(i1)*2*pi)^2]);

      % convert continuous TF to z domain
      % FOH seems to work best per from SpectralA04_check.m
%     filt1d=c2d(filt1,dt,'zoh');
%     filt1d=c2d(filt1,dt,'prewarp',f(i1)*2*pi);
      filt1d=c2d(filt1,dt,'foh');
%     temp=get(filt1d);  %lists structure fieldnames
%      b=filt1d.num{1}; % extract coeffs (Octave compatible)
%      a=filt1d.den{1};

      [b, a] = tfdata(filt1d,'v'); % extract coeffs (Octave/Matlab compatible)

      for i2=1:ns  % filter signals
        Sa(i1,i2)=max(abs(filter(b,a,InputSignal(:,i2))));
      end

    end
end

%% Function to compute mode frequency response
%%
function G_jw = freqresp_2ndorder_model(phiB,phiC,om,om0,damp,ind_in,ind_out)
% G_jw = freqresp_2ndorder_model(phiB,phiC,om,om0,damp,ind_in,ind_out)
% * om is vector of frequencies to apply bode at (rad/s)
% * om0 is a vector of natural frequencies (rad/s)
% * damp is a vector of damping ratios (0-1)
% * ind_in and ind_out are input and output indices requested.
% * phiB and phiC are the nonzero partitions of the input and output
% matrices of the state-space model.
%

% Trim input/output matrices as appropriate.
phiB = phiB(:,ind_in);
phiC = phiC(ind_out,:);
%D = D(ind_out,ind_in);

% Define useful dimensions.
[n_m, n_u] = size(phiB);
n_y = size(phiC,1);
n_om = length(om);

s = sqrt(-1)*om;
% Initialize output
G_jw = zeros(n_y, n_u, n_om);

for i =1:n_y %loop through outpus
    for j = 1:n_u % loop through inputs
        for k = 1:n_m %loop through modes
            G_jw(i,j,:) = G_jw(i,j,:)+...
                phiC(i,k)*...
                reshape(1./(s.^2+2*damp(k)*om0(k).*s+om0(k).^2),1,1,n_om)*...
                phiB(k,j);
        end
        %G_jw(i,j,:) = G_jw(i,j,:) + D(i,j);
    end
end

end









