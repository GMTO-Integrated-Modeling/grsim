%% RLE SSSHA Assessment

reload_model = false;

% dt_file = {'./data/model-RLE01_wiM1c_noSGMC_1.parquet';...
%     './data/model-RLE01_wiM1c_wiSGMC_1.parquet';...
%     './data/model-RLE01_woM1c_noSGMC_1.parquet';...
%     './data/model-data_1.parquet'};

% dt_file = './data/model-RLE01_wiM1c_noSGMC_1.parquet';
dt_file = './data/model-RLE01_woM1c_noSGMC_1.parquet';
% dt_file = './data/model-RLE01_wiM1c_wiSGMC_1.parquet';

%%
% Ts = diff(t(1:2));

try
    sssha_data = parquetread(dt_file,"SampleRate",1e3,...
        "SelectedVariableNames",{'OSS00GroundAcc','OSSHardpointD','MountEncoders'}); %'OSSPayloads6D'
catch
    warning('Unable to run parquetread(). Try Matlab 2022b, or later.');
end

t = seconds(sssha_data.Time);
HP_D = reshape(cell2mat(sssha_data.OSSHardpointD),84,[]);
gnd_acc = reshape(cell2mat(sssha_data.OSS00GroundAcc),3,[]);
mnt_enc = reshape(cell2mat(sssha_data.MountEncoders),14,[]);

%% Mount ENC plot
%%
t_range = [];%[18.23,18.238];%[4.19,4.208];%[];%
if(isempty(t_range))
    t_idx  = [1, length(t)];
else
    t_idx = [find(t >= t_range(1),1,"first"),find(t < t_range(2),1,"last")];
end

if false
    plot(t(t_idx(1):t_idx(2)),gnd_acc(:, t_idx(1):t_idx(2))');
    ylabel('Accelerations (m/s^2)');
    xlabel('Time (s)');
    legend('H1','H2','V');
    grid on; axis tight;
end

if(exist('mnt_enc','var'))
    figure(1)
    az_enc = mean(mnt_enc(1:4,t_idx(1):t_idx(2)))';
    el_enc = mean(mnt_enc(5:10,t_idx(1):t_idx(2)))';
    gir_enc = mean(mnt_enc(11:14, t_idx(1):t_idx(2)))';
    plot(t(t_idx(1):t_idx(2)),[az_enc, el_enc, gir_enc]);
    ylabel('MNT ENC (rad)');
    xlabel('Time (s)');
    legend('AZ','EL','GIR');
    grid on; axis tight;
end


%% Structural model data 
%% 

ModelFolder = "20240408_1535_zen_30_M1_202110_FSM_202305_Mount_202305_IDOM_concreteAndFoundation_finalSI_largeMass";
FileName = "modal_state_space_model_2ndOrder.mat";
    
if(~exist('inputTable','var') || reload_model)    
    load(fullfile(im.lfFolder,ModelFolder,FileName),...
        'inputs2ModalF','modalDisp2Outputs',...
        'eigenfrequencies','inputTable','outputTable');

    fprintf('Model %s loaded from\n%s\n', FileName, ModelFolder);
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

%% HP stiffness
%%
i_in = inputTable{"OSS_Harpoint_delta_F","indices"}{1};
i_out = outputTable{"OSS_Hardpoint_D","indices"}{1};
T_hp = kron(eye(7),[-eye(6),eye(6)]);

HPk = 1./ diag(T_hp*gainMatrix(i_out,i_in));
fprintf('HP stiffness: (avg) %g (N/um), std:%g\n', mean(1e-6*HPk), std(1e-6*HPk))
if(exist('eigenfrequencies','var'))
    invOM2 = diag(1./((2*pi*eigenfrequencies).^2));
    HPk_modal = 1./diag(T_hp * modalDisp2Outputs(i_out,:) * invOM2 * inputs2ModalF(:,i_in));
    fprintf('HP stiffness (from modal model): (avg) %g (N/um), std:%g\n', mean(1e-6*HPk_modal), std(1e-6*HPk_modal))
end

% HP stiffness due to mode truncation
if false
    freq_lim_vec = (45:1:max(eigenfrequencies))'; %#ok<*UNRCH> 
    HPk_dyn = zeros(numel(freq_lim_vec),2);
    for iw = 1:numel(freq_lim_vec)
        maxf_i = find(eigenfrequencies <= freq_lim_vec(iw),1,"last");
        invOM2_ = diag(1./((2*pi*eigenfrequencies(1:maxf_i)).^2));
        HPk_ = 1./diag(T_hp * modalDisp2Outputs(i_out,1:maxf_i) * invOM2_ * inputs2ModalF(1:maxf_i,i_in));
        HPk_dyn(iw,:) = [mean(HPk_), std(HPk_)];
    end
    figure
    plot(freq_lim_vec,HPk_dyn(:,1),'s:');
    hold on; semilogy(max(eigenfrequencies),mean(HPk_modal),'x');
    grid on;
    xlabel('Max mode frequency (Hz)');
    ylabel('HP stiffness (N/m)')
end

%% HP axial forces
%%
HPk_ = mean(HPk_modal); %mean(HPk)
HP_axialF = HPk_ * T_hp * HP_D;
% HP_raxialD = T_hp * HP_D;

% M1 HP breakaway force limits
f_compression = 1200;
f_tension = 900;

t_range = [];%[];%[4.19,4.203];%
if(isempty(t_range))
    t_idx  = [1, length(t)];
else
    t_idx = [find(t >= t_range(1),1,"first"),find(t < t_range(2),1,"last")];
end
figure
tk = t(t_idx(1):t_idx(2));
plot(tk, HP_axialF(:,t_idx(1):t_idx(2))');
hold on; plot(tk, f_compression*ones(size(tk)),'k--');
plot(tk, -f_tension*ones(size(tk)),'k--'); 
grid on; hold off;
text(round(0.98*max(tk)), f_compression,...
    sprintf('Breakaway compression limit (%dN)',f_compression),...
    'VerticalAlignment','bottom', 'HorizontalAlignment','right');
text(round(0.98*max(tk)), -f_tension,...
    sprintf('Breakaway tension limit (%dN)',f_tension),...
    'VerticalAlignment','top', 'HorizontalAlignment','right');
ylabel(sprintf('HP Forces (N) - HP stiffness=%3.1fN/um', mean(HPk_)/1e6));
%ylabel('HP axial displacement (m)');
xlabel('Time (s)'); axis tight;
% grid on; axis tight;
SE = [max(xlim) min(ylim)]-[diff(xlim) -diff(ylim)]*0.05;
dt_file_label = replace(extractAfter(dt_file,"data/"),'_','\_');
text(SE(1), SE(2), dt_file_label,'Fontsize',12,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','right')

