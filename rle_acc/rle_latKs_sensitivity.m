%
% rle_latKs_sensitivity.m
%
% Description: the script assesses the effect of the lateral stiffness of
% the seismic isolation system on the pier acceleration.
%

% - Mount the s3 volume using:
% mount-s3 --cache ~/.s3-cache/ gmto.im.grim ~/mnt
% - Update s3_path variable below


%% General Settings
%%
s3_path = "/home/rromano/mnt";
dt_fname_prot = "model-%s-RLE0%d_wiM1c_wiSGMC_1.parquet";
rle_cases = (1:7);
struct_models = [...
    "20240408_1535";...
    "20241003_1800";...
    "20230817_1808";...
    "20241021_1535"];

show_gndACC = true;

%% Spectral Acceleration Settings
%%
% Simulation data sampling frequency
fs = 1e3;
% [SRS_STRUCT] = compute_response_spectra (EQ_STRUCT, ZETA)
% B. Smith, 18 Sept 2024
zeta = .02; % payload damping ratio
STEPS = 4; % number of frequency steps per FWHM, 4 yields <3.6% scalloping
FMIN = 0.5; %  minimum frequency [Hz]

dT = 1/fs;
fmax = fs/2;
q = 1/(2*zeta); % resonance Q factor, Q = f/FWHM

nFreqs = round(log(fmax/FMIN)/log(1+1/(q*STEPS)));
fSRS = logspace(log10(FMIN),log10(fmax),nFreqs); % frequency vector [Hz]

%% 
% preallocate result
pierSHA = zeros(nFreqs, 7, numel(struct_models));
gndSHA = zeros(nFreqs, 7, numel(struct_models));

tic
for k_model = 1:numel(struct_models)
    model_label_id = struct_models(k_model);
    model_folder = dir(fullfile(s3_path,...
        sprintf('*%s*largeMass',model_label_id)));
    %  Loop over the RLE cases
    parfor i_rle = 1:numel(rle_cases)
        sssha_data = [];
        dt_file = fullfile(s3_path, model_folder.name,...
            sprintf(dt_fname_prot,model_label_id,rle_cases(i_rle)));
        fprintf("Post-processing data from %s\n",dt_file);
        try
            parquetINFO = parquetinfo(dt_file);
            sssha_data = parquetread(dt_file,"SampleRate",1e3,...
                "SelectedVariableNames",parquetINFO.VariableNames);
            % "OSSPayloads6D";"OSS00GroundAcc";"OSSHardpointD";"OSSM1Lcl";"MountEncoders"
        catch
            warning('Unable to run parquetread(). Try Matlab 2022b, or later.');
        end

        t = seconds(sssha_data.Time);
        assert(dT, diff(t(1:2)));
        pier_D = reshape(cell2mat(sssha_data.Pier6D),12,[]);
        gnd_ACC = reshape(cell2mat(sssha_data.OSS00GroundAcc),3,[]);
        gnd_D = reshape(cell2mat(sssha_data.OSS00Ground6D),6,[]);

        % Numerical derivative to calculate the pier node acceleration
        pier_acc_dt = 1/dT*diff(1/dT*diff(pier_D(1:3,:)'));
        %
        gnd_acc_dt = 1/dT*diff(1/dT*diff(gnd_D(1:3,:)'));
        sa_data = zeros(nFreqs, 2); % preallocate SA result
        gnd_sa_data = zeros(nFreqs, 2); % preallocate SA result

        for j_aax = 1:2 % 1:H1, 2:H2
            sa_data(:,j_aax) = SpectralA04(pier_acc_dt(:,j_aax), fSRS, dT, zeta);        

            if(show_gndACC) %k_model==1
                gnd_sa_data(:,j_aax) = SpectralA04(gnd_acc_dt(:,j_aax), fSRS, dT, zeta);

%                 if(j_aax==1), tmp1 = SpectralA04(gnd_ACC(:,1), fSRS, dT, zeta);
%                 else
%                     gndSHA(:,i_rle) = exp(mean(log([...
%                         tmp1,SpectralA04(gnd_ACC(:,2), fSRS, dT, zeta)]...
%                         ),2));
%                 end
            end
        end
        gndSHA(:,i_rle,k_model) = exp(mean(log(gnd_sa_data),2));
        pierSHA(:,i_rle,k_model) =  exp(mean(log(sa_data),2));
    end
end
toc

%% Plot SA Results
%%
sa_g = squeeze(exp(mean(log(pierSHA),2))/9.80665);

hsa_fig = figure(1);
% 15Hz vertical line
freq_line = 5;
hfline1 = loglog(freq_line*[1, 1],[1e-3 1e3],':');
text(freq_line, 0.02, sprintf('%dHz ',freq_line),'HorizontalAlignment','right');
hold on;
freq_line = 15;
patch([5, 5, 15, 15],[1e-3 1e3 1e3 1e-3], [1 0 0]*0.8, 'EdgeColor','none', 'FaceAlpha',0.1);
hfline2 = loglog(freq_line*[1, 1],[1e-3 1e3],':');
text(freq_line, 0.02, sprintf(' %dHz',freq_line),'HorizontalAlignment','left');
set(gca,'ColorOrderIndex',1);

if(show_gndACC)
    gnd_sa_g = squeeze(exp(mean(log(gndSHA),2))/9.80665);
    h_gnd = loglog(fSRS,gnd_sa_g(:,1),'-');%,'color',0.5*[1,1,1]);
    legends = "Ground SA (lateral components)";
else
    legends = []; h_gnd=[]; %#ok<*UNRCH> 
end

h_ax1 = loglog(fSRS,sa_g(:,[3,4]));
set(gca,'ColorOrderIndex',5);
h_ax2 = loglog(fSRS,sa_g(:,[1,2]),'--','Linewidth',1.5);
ylim([.01 2]); xlim([0.8, 100]);
legends = [legends,...
    "REQ lateral stiffness: 3e9N/m",...
    "Increased lateral stiffness: 9e9N/m",...
    "Lateral stiffness: 0.96e9N/m",...
    "Rad & Tang stiffness \rightarrow 1.36e9N/m"];
legend([h_gnd; h_ax1; h_ax2], legends);
title(sprintf('RLE Stiffness Sensitiviy (zeta=%.3g)',zeta));
xlabel('Frequency (Hz)');ylabel('(B) Pier Horizontal Acc (g)');
grid on;
hold off;
exportgraphics(hsa_fig, 'pierSHA_latKs_sens.png', 'Resolution', 300); 




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