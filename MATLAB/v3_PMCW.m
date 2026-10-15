% PMCW Tracking Radar - Detectability Analysis
% Adapted from MATLAB Radar Toolbox R2026a TrackingRadar export
%
% Waveform: Phase-Modulated Continuous Wave (PMCW)
%
% Mapping to pulsed-radar framework:
%   Chip duration (1/B)        → Pulsewidth  (single-sample energy)
%   Chips per CPI (B × T_int) → NumCoherentPulses (correlation gain)
%   Average power              → PeakPower   (100% duty cycle)
%
% Generated on: 03-Jul-2026

%% PMCW Waveform Summary
pmcw.Bandwidth       = 15e6;                          % Hz (code bandwidth)
pmcw.ChipDuration    = 1 / pmcw.Bandwidth;             % s  (~66.7 ns)
pmcw.IntegrationTime = 10e-3;                          % s  (coherent processing interval)
pmcw.NumChips        = pmcw.Bandwidth * pmcw.IntegrationTime; % 150 000 chips/CPI
pmcw.AvgPower        = 125;                            % W  (continuous Tx power)
pmcw.CodeLength      = 1023;                           % chips (Gold code period)
pmcw.CodePeriod      = pmcw.CodeLength / pmcw.Bandwidth; % s  (~68.2 us)
pmcw.ProcessingGain  = 10*log10(pmcw.NumChips);        % dB (~51.8 dB)

c = physconst('LightSpeed');
fprintf('========== PMCW Waveform Parameters ==========\n');
fprintf('Bandwidth:          %.0f MHz\n',   pmcw.Bandwidth/1e6);
fprintf('Chip Duration:      %.2f ns\n',    pmcw.ChipDuration*1e9);
fprintf('Integration Time:   %.1f ms\n',    pmcw.IntegrationTime*1e3);
fprintf('Chips per CPI:      %d\n',         pmcw.NumChips);
fprintf('Processing Gain:    %.2f dB\n',    pmcw.ProcessingGain);
fprintf('Code Length:         %d chips (Gold code)\n', pmcw.CodeLength);
fprintf('Code Period:         %.2f us\n',   pmcw.CodePeriod*1e6);
fprintf('Average Power:       %.0f W\n',    pmcw.AvgPower);
fprintf('Range Resolution:    %.2f m\n',    c/(2*pmcw.Bandwidth));
fprintf('Vel  Resolution:     %.2f m/s\n',  freq2wavelen(10e9)/(2*pmcw.IntegrationTime));
fprintf('Unamb Range (code):  %.0f m\n',    c*pmcw.CodePeriod/2);
fprintf('Unamb Velocity:      %.1f m/s\n',  freq2wavelen(10e9)/(2*pmcw.CodePeriod));
fprintf('================================================\n\n');

%% Set up radar, environment and target
radar  = designRadarPMCW(pmcw);
env    = radarEnvironment();
rangeLimits = [1e-2  2e3]; % m
target = radarTarget(radar, env, rangeLimits);

%% Constraints and design requirements
constraint.Range = 1000;       % m
PdThreshold      = 0.70;
PdObjective      = 0.95;

%% Propagation and losses
Fprop = propagationFactor(radar, env, target);
[Lgas, Llens, Lprecipitation] = atmosphericLosses(radar, env, target);
Latmos  = sum([Lgas Llens Lprecipitation], 2);
Fcustom = rangeDependentLosses(radar, target);

%% Available single-chip SNR vs range
availableSNR = radareqsnr(radar.Wavelength, target.ranges, radar.PeakPower, ...
    radar.Pulsewidth, ...
    'Gain',              radar.Gain, ...
    'RCS',               target.RCS, ...
    'Ts',                radar.SystemTemperature, ...
    'PropagationFactor',  Fprop, ...
    'AtmosphericLoss',    Latmos, ...
    'CustomFactor',       Fcustom); % dB  (single-chip)

%% Detectability (includes PMCW correlation gain internally)
requiredPd = [PdThreshold PdObjective];
[Dx, Dc, componentNames] = effectiveDetectability(requiredPd, radar, target);

% Pd vs range
probabilityGrid = probgrid(0.1, 1-1e-6, 100);
requiredSNR     = effectiveDetectability(probabilityGrid, radar, target);
[~, idx]   = unique(requiredSNR);
idxNotInf  = ~isinf(availableSNR);
availablePd = zeros(size(availableSNR));
if any(idxNotInf)
    availablePd(idxNotInf) = rocinterp(requiredSNR(idx), probabilityGrid(idx), ...
        availableSNR(idxNotInf), 'snr-pd');
end
availablePd(availableSNR > requiredSNR(end)) = 1;
availablePd(availableSNR < requiredSNR(1))   = 0;

%% ===================== PLOT 1: SNR vs Range =====================
figure('Name','PMCW - SNR vs Range','Position',[50 550 820 480]);
hAxes = gca;
radarmetricplot(target.ranges*1e-3, availableSNR, Dx(2), Dx(1), ...
    'MaxRangeRequirement', constraint.Range*1e-3, ...
    'RadarName',          {radar.Name}, ...
    'MetricName',         'SNR', ...
    'RequirementName',    'Detectability', ...
    'RangeUnit',          'km', ...
    'MetricUnit',         'dB', ...
    'ShowStoplight',      true, ...
    'Parent',             hAxes);
title('PMCW Tracking Radar — SNR vs Target Range');

%% ===================== PLOT 2: Pd vs Range ======================
figure('Name','PMCW - Pd vs Range','Position',[50 50 820 480]);
hAxes = gca;
radarmetricplot(target.ranges*1e-3, availablePd, PdObjective, PdThreshold, ...
    'MaxRangeRequirement', constraint.Range*1e-3, ...
    'RadarName',          {radar.Name}, ...
    'MetricName',         'Probability of Detection', ...
    'RequirementName',    'Pd', ...
    'RangeUnit',          'km', ...
    'ShowStoplight',      true, ...
    'Parent',             hAxes);
ylim(hAxes,[0.1 1.04]);
title('PMCW Tracking Radar — Pd vs Target Range');

%% ===================== PLOT 3: Link Budget ======================
figure('Name','PMCW - Link Budget','Position',[900 550 900 580]);
subplot(2,1,1);
radarbudgetplot(Dc(2,:), componentNames);
title(sprintf('Objective Detectability  (P_d = %.2f)', PdObjective));

subplot(2,1,2);
radarbudgetplot(Dc(1,:), componentNames);
title(sprintf('Threshold Detectability  (P_d = %.2f)', PdThreshold));
sgtitle([radar.Name ' — PMCW Link Budget']);

%% ============= PLOT 4: Range-Doppler Coverage ===================
figure('Name','PMCW - Range-Doppler Coverage','Position',[900 50 900 580]);

% Axes
rangeRes       = c / (2*pmcw.Bandwidth);                 % 10 m
unambRange     = c * pmcw.CodePeriod / 2;                % ~10 230 m
maxPlotRange   = min(unambRange, constraint.Range);      % 1 000 m
velRes         = radar.Wavelength / (2*pmcw.IntegrationTime); % ~1.5 m/s
maxUnambVel    = radar.Wavelength / (2*pmcw.CodePeriod);     % ~220 m/s

rangeAxis = linspace(10, maxPlotRange, 500)';
velAxis   = linspace(-maxUnambVel, maxUnambVel, 501)';

% Single-chip SNR across range (no propagation for clean R-D view)
snr1chip = radareqsnr(radar.Wavelength, rangeAxis, radar.PeakPower, ...
    radar.Pulsewidth, ...
    'Gain', radar.Gain, 'RCS', target.RCS, 'Ts', radar.SystemTemperature);

% Total SNR after coherent correlation gain
totalSNR = snr1chip + pow2db(radar.NumCoherentPulses); % dB

% Doppler mismatch loss (sinc² roll-off within unambiguous Doppler)
[R_grid, V_grid] = meshgrid(rangeAxis, velAxis);
normVel     = V_grid / maxUnambVel;          % normalised to ±1
dopplerLoss = 20*log10(max(abs(sinc(normVel)), 1e-3)); % negative values = loss

% Build 2-D SNR map
SNR_map = repmat(totalSNR', length(velAxis), 1) + dopplerLoss;

% --- subplot 1: SNR heat-map ---
ax1 = subplot(2,1,1);
imagesc(rangeAxis, velAxis, SNR_map);
set(ax1,'YDir','normal');
hold on;
contour(R_grid, V_grid, SNR_map, [Dx(1) Dx(1)], 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Pd_{thr}=%.0f%%', PdThreshold*100));
contour(R_grid, V_grid, SNR_map, [Dx(2) Dx(2)], 'w--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Pd_{obj}=%.0f%%', PdObjective*100));
colorbar; colormap(ax1, jet);
caxis([0 max(totalSNR)]);
xlabel('Range (m)'); ylabel('Radial Velocity (m/s)');
title('Range-Doppler SNR (dB)');
legend('Location','northeast');
hold off;

% --- subplot 2: detection mask ---
ax2 = subplot(2,1,2);
detMask = double(SNR_map >= Dx(1));
imagesc(rangeAxis, velAxis, detMask);
set(ax2,'YDir','normal');
colormap(ax2, [0.15 0.15 0.15; 0.18 0.75 0.25]);
xlabel('Range (m)'); ylabel('Radial Velocity (m/s)');
title(sprintf('Detection Coverage  (green = P_d \\geq %.0f%%)', PdThreshold*100));

sgtitle('PMCW Tracking Radar — Range-Doppler Coverage');

annotation('textbox',[0.12 0.005 0.78 0.035], ...
    'String', sprintf(['\\DeltaR = %.1f m  |  \\Deltav = %.2f m/s  |  ' ...
    'R_{unamb} = %.0f m  |  v_{unamb} = \\pm%.0f m/s'], ...
    rangeRes, velRes, unambRange, maxUnambVel), ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',9);

%% Detection-range summary
fprintf('\n========== Detection Performance ==========\n');
idxThr = find(availablePd >= PdThreshold, 1, 'last');
idxObj = find(availablePd >= PdObjective, 1, 'last');
if ~isempty(idxThr)
    fprintf('Max range at Pd >= %.0f%%:  %.0f m\n', PdThreshold*100, target.ranges(idxThr));
end
if ~isempty(idxObj)
    fprintf('Max range at Pd >= %.0f%%:  %.0f m\n', PdObjective*100, target.ranges(idxObj));
end
fprintf('============================================\n');

%% ==================== LOCAL FUNCTIONS ====================

% ---- Radar (PMCW-equivalent) ----
function radar = designRadarPMCW(pmcw)
    radar.Name = 'PMCW TrackingRadar';

    % Waveform equivalence
    radar.Frequency          = 10e9;                  % Hz
    radar.PulseBandwidth     = pmcw.Bandwidth;        % Hz
    radar.Pulsewidth         = pmcw.ChipDuration;     % s  (single chip)
    radar.PeakPower          = pmcw.AvgPower;         % W  (CW → peak=avg)
    radar.NumCoherentPulses  = pmcw.NumChips;          % correlation gain
    radar.NumNonCoherentPulses = 1;
    radar.PRF                = pmcw.Bandwidth;        % chip rate

    % Antenna (4×[8×8] = 256 elements, sinc pattern)
    radar.AzimuthBeamwidth   = [15; 15];              % deg
    radar.ElevationBeamwidth = [10; 10];              % deg
    radar.Gain = [23.3445375115093, 23.3445375115093];% dBi
    radar.Height   = 1.70;                            % m
    radar.TiltAngle = 0;                              % deg
    radar.Polarization = 'V';

    % Detection & CFAR
    radar.Pfa          = 1e-4;
    radar.CFARMethod   = 'GOCA';     % Greatest-Of Cell Averaging
    radar.CFARNumCells = 28;
    radar.NumCPIs      = 2;          % processing intervals per update
    radar.MNumCPIs     = 1;          % M-of-N: detect in >= 1 of 2

    % Electronic scan limits
    radar.AzScanStartElectronic = -10;
    radar.AzScanStopElectronic  =  10;
    radar.ElScanStartElectronic =   0;
    radar.ElScanStopElectronic  =  89;

    % Losses
    radar.CustomLoss     = 3;        % dB  (Tx/Rx isolation + misc)
    radar.STCCutOffRange = 1000;     % m
    radar.STCExponent    = 3.5;

    % Noise
    radar.NoiseTemperature          = 578.6261;       % K  (NF=3 dB, T0=290 K)
    radar.ReferenceNoiseTemperature = 290;
    radar.SystemTemperature         = radar.NoiseTemperature;

    % Derived
    radar.Wavelength       = freq2wavelen(radar.Frequency);
    radar.MaxSpeed         = 187.37;
    radar.RangeRateRes     = 1.46383;
    radar.UnambiguousRange = physconst('LightSpeed') * pmcw.CodePeriod / 2;

    radar.EffectiveAzRes = effbeamwidth(radar.AzimuthBeamwidth(1), ...
                                        radar.AzimuthBeamwidth(2));
    radar.EffectiveElRes = effbeamwidth(radar.ElevationBeamwidth(1), ...
                                        radar.ElevationBeamwidth(2));
end

% ---- Environment ----
function env = radarEnvironment()
    env.EarthModel = 'Curved';
    env.LatitudeModel = 'Standard';
    env.SurfaceHeightStandardDeviation = 0;
    env.SurfaceRelativePermittivity = -0 - 1i*1e16;
    env.SurfaceSlope = 0;
    env.VegetationType = 'None';

    [env.RefractiveIndex, env.SurfaceRefractivity] = refractiveidx(0, ...
        'LatitudeModel', env.LatitudeModel);
    env.RefractionExponent = refractionexp(env.SurfaceRefractivity);

    n = refractiveidx([0 1e3], 'LatitudeModel', env.LatitudeModel);
    RGradient = (n(2)-n(1)) / 1e3;
    env.EffectiveEarthRadius = effearthradius(RGradient);
end

% ---- Target ----
function target = radarTarget(radar, env, rangeLimits)
    target.ElevationAngle = 5;          % deg
    target.RCS            = 1e-3;       % m²  (micro-UAV)
    target.SwerlingCase   = 'Swerling1';

    numPts = 1000;
    target.ranges = linspace(rangeLimits(1), rangeLimits(2), numPts).';
    target.validRngIdxs = getValidRangeIdxs(radar, target, env);
    target.Height = el2height(target.ElevationAngle, radar.Height, ...
        target.ranges, env.EarthModel, env.EffectiveEarthRadius);
end

% ---- Propagation factor ----
function Fprop = propagationFactor(radar, env, target)
    Fprop = -Inf(numel(target.ranges), 2);
    ok = target.Height >= env.SurfaceHeightStandardDeviation;
    for ie = 1:2
        Fprop(ok, ie) = radarpropfactor( ...
            target.ranges(ok), radar.Frequency, radar.Height, target.Height(ok), ...
            'Polarization',                  radar.Polarization, ...
            'SurfaceRelativePermittivity',   env.SurfaceRelativePermittivity, ...
            'SurfaceHeightStandardDeviation',env.SurfaceHeightStandardDeviation, ...
            'SurfaceSlope',                  env.SurfaceSlope, ...
            'VegetationType',                env.VegetationType, ...
            'ElevationBeamwidth',            radar.ElevationBeamwidth(ie), ...
            'TiltAngle',                     radar.TiltAngle, ...
            'EffectiveEarthRadius',          env.EffectiveEarthRadius, ...
            'RefractiveIndex',               env.RefractiveIndex);
    end
    Fprop(~ok,:) = -inf;
end

% ---- Atmospheric losses ----
function [Lgas, Llens, Lrain] = atmosphericLosses(radar, env, target)
    N = numel(target.ranges);
    Lgas  = zeros(N,1);
    Llens = zeros(N,1);
    Lrain = zeros(N,1);

    idxs = target.validRngIdxs;
    elArr = target.ElevationAngle * ones(size(idxs));

    Lgas(idxs)  = tropopl(target.ranges(idxs), radar.Frequency, ...
                           radar.Height, elArr, 'LatitudeModel', env.LatitudeModel);
    Llens(idxs) = lenspl(target.ranges(idxs), radar.Height, elArr, ...
                          'LatitudeModel', env.LatitudeModel);
end

% ---- STC / custom range losses ----
function F = rangeDependentLosses(radar, target)
    F = stcfactor(target.ranges, radar.STCCutOffRange, radar.STCExponent);
end

% ---- Valid range indices ----
function idxs = getValidRangeIdxs(radar, target, env)
    maxRng = 2*env.EffectiveEarthRadius + inf + ...
             radar.Height + 2*env.SurfaceHeightStandardDeviation;
    idxs = (target.ranges >= 0 & target.ranges <= maxRng);
end

% ---- Effective detectability ----
function [Dx, Dc, names, pg] = effectiveDetectability(Pd, radar, target)
    Pfa = radar.Pfa;
    [Dc, names] = detectabilityComponents(Pd, Pfa, ...
        radar.NumNonCoherentPulses, target.SwerlingCase, radar);
    Dx = sum(Dc, 2);

    [~, pd, pfa] = binaryintloss(Pd, Pfa, radar.NumCPIs, radar.MNumCPIs);
    pd  = max(pd,  0.10);
    pfa = min(pfa, 0.001);
    Dccpi = detectabilityComponents(pd, pfa, ...
        radar.NumNonCoherentPulses, target.SwerlingCase, radar);
    Gcpi = sum(Dccpi, 2) - Dx;
    Dx   = Dx + Gcpi;
    Dc   = [Dc Gcpi];
    names = [names  'CPI integration gain (2nd interval)'];
    pg   = -sum(Dc(:,2:end), 2);
end

% ---- Detectability components ----
function [Dc, names] = detectabilityComponents(Pd, Pfa, N, SW, radar)
    % 1) Single-chip steady-target detectability
    D01 = detectability(Pd, Pfa, 1, 'Swerling0');

    % 2) Fluctuation loss (Swerling 1)
    Lf = detectability(Pd, Pfa, N, SW) ...
       - detectability(Pd, Pfa, N, 'Swerling0');

    % 3) PMCW correlation gain  =  10·log10(NumChips)
    %    In the pulsed framework this is "coherent integration gain"
    Gci = -pow2db(radar.NumCoherentPulses) * ones(numel(Pd),1);

    % 4) GO-CFAR loss
    Lcfar = cfarloss(Pfa, radar.CFARNumCells, 'Method', radar.CFARMethod) ...
            * ones(numel(Pd),1);

    % 5) Beam-shape loss
    Lbeam = beamloss(true) * ones(numel(Pd),1);

    % 6) Array electronic-scan loss
    Lscan = arrayscanloss(Pd, Pfa, N, ...
                [radar.AzScanStartElectronic radar.AzScanStopElectronic], SW) ...
          + arrayscanloss(Pd, Pfa, N, ...
                [radar.ElScanStartElectronic radar.ElScanStopElectronic], SW);

    % 7) Custom loss (Tx/Rx isolation + cable/connector)
    Lcustom = radar.CustomLoss * ones(numel(Pd),1);

    Dc    = [D01  Lf  Gci  Lcfar  Lbeam  Lscan  Lcustom];
    names = {'Single-chip detectability', ...
             'Swerling-1 fluctuation loss', ...
             'PMCW correlation gain (B·T_{int})', ...
             'GO-CFAR loss', ...
             'Beam-shape loss', ...
             'Array electronic-scan loss', ...
             'Custom loss (Tx/Rx isolation)'};
end