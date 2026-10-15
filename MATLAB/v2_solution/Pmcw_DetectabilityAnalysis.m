% =========================================================================
% DETECTABILITY ANALYSIS — PMCW Radar for Small Drone Detection at 1 km
% =========================================================================
% Corrected from Radar Designer v2 export
% Key changes from v2:
%   - PMCW: Pulsewidth = 1/PRF (continuous TX, duty=1.0) → +3 dB
%   - Target elevation: 60° → 0° (worst case propagation)
%   - Swerling: 0 → 1 (realistic fluctuating drone RCS)
%   - RCS: 0.01 m² (-20 dBsm) for small drone
%   - Rain: 10 mm/hr moderate (not 42 mm/hr torrential)
%
% Generated on: 02-Jul-2026 (corrected)
% =========================================================================

%% Set up radar, target, and environment structures
radar = designRadar('DroneDetectionRadar_PMCW');
env   = radarEnvironment();

% Range limits for analysis
rangeLimits = [1e-2 2000]; % m (10 mm to 2 km)
target = radarTarget(radar, env, rangeLimits);

%% Define constraints and design requirements
constraint.Range = 1000; % m (1 km)
PdThreshold = 0.50;
PdObjective = 0.92;

% Calculate propagation factor
Fprop = propagationFactor(radar, env, target);

% Calculate environmental losses
[Lgas, Llens, Lprecipitation] = atmosphericLosses(radar, env, target);
Latmos = sum([Lgas Llens Lprecipitation], 2);

% Calculate range-dependent factors (STC)
Fcustom = rangeDependentLosses(radar, target);

%% Compute available SNR
availableSNR = radareqsnr(radar.Wavelength, target.ranges, radar.PeakPower, ...
    radar.Pulsewidth, ...
    'Gain', radar.Gain, ...
    'RCS', target.RCS, ...
    'Ts', radar.SystemTemperature, ...
    'PropagationFactor', Fprop, ...
    'AtmosphericLoss', Latmos, ...
    'CustomFactor', Fcustom); % dB

%% Calculate detectability
requiredPd = [PdThreshold PdObjective];
[Dx, Dc, componentNames] = effectiveDetectability(requiredPd, radar, target);

% Probability of detection vs range
probabilityGrid = probgrid(0.1, 1-1e-6, 100);
requiredSNR = effectiveDetectability(probabilityGrid, radar, target);
[~, idx] = unique(requiredSNR);
idxNotInf = ~isinf(availableSNR);
availablePd = zeros(size(availableSNR));
if any(idxNotInf)
    availablePd(idxNotInf) = rocinterp(requiredSNR(idx), probabilityGrid(idx), availableSNR(idxNotInf), 'snr-pd');
end
availablePd(availableSNR > requiredSNR(end)) = 1;
availablePd(availableSNR < requiredSNR(1))   = 0;

%% Print key results
fprintf('=== PMCW Drone Detection Radar — Detectability ===\n');
fprintf('Waveform:            PMCW (Phase-Modulated Continuous Wave)\n');
fprintf('Frequency:           %.1f GHz\n', radar.Frequency/1e9);
fprintf('Bandwidth:           %.1f GHz\n', radar.PulseBandwidth/1e9);
fprintf('Peak Power:          %.2f W\n', radar.PeakPower);
fprintf('Duty Cycle:          1.0 (continuous)\n');
fprintf('Coherent Pulses:     %d\n', radar.NumCoherentPulses);
fprintf('Target RCS:          %.2f dBsm (%.4f m²)\n', 10*log10(target.RCS), target.RCS);
fprintf('Target Elevation:    %.1f°\n', target.ElevationAngle);
fprintf('Swerling Model:      %s\n', target.SwerlingCase);
fprintf('Rain Rate:           %d mm/hr\n', env.RainRate);
rangeIdx = find(target.ranges >= constraint.Range, 1);
if ~isempty(rangeIdx)
    fprintf('SNR at %.0f m:        %.1f dB\n', constraint.Range, availableSNR(rangeIdx));
    fprintf('Pd at %.0f m:         %.3f\n', constraint.Range, availablePd(rangeIdx));
end
fprintf('Threshold Detect.:   %.1f dB\n', Dx(1));
fprintf('Objective Detect.:   %.1f dB\n', Dx(2));
fprintf('==================================================\n');

%% Plot SNR vs Range
figure('Name', 'SNR vs Range — PMCW Drone Detection');
hAxes = gca;
radarmetricplot(target.ranges*1e-3, availableSNR, Dx(2), Dx(1), ...
    'MaxRangeRequirement', constraint.Range*1e-3, ...
    'RadarName', {radar.Name}, ...
    'MetricName', 'SNR', ...
    'RequirementName', 'Detectability', ...
    'RangeUnit', 'km', ...
    'MetricUnit', 'dB', ...
    'ShowStoplight', true, ...
    'Parent', hAxes);

%% Plot Pd vs Range
figure('Name', 'Pd vs Range — PMCW Drone Detection');
hAxes = gca;
radarmetricplot(target.ranges*1e-3, availablePd, PdObjective, PdThreshold, ...
    'MaxRangeRequirement', constraint.Range*1e-3, ...
    'RadarName', {radar.Name}, ...
    'MetricName', 'Probability of Detection', ...
    'RequirementName', 'Pd', ...
    'RangeUnit', 'km', ...
    'ShowStoplight', true, ...
    'Parent', hAxes);
ylim(hAxes, [0.1 1.04]);

%% Link Budget
figure('Name', 'Link Budget — PMCW Drone Detection');
subplot(2,1,1);
radarbudgetplot(Dc(2,:), componentNames);
title(sprintf('Objective Detectability (P_d=%.2f)', PdObjective));
subplot(2,1,2);
radarbudgetplot(Dc(1,:), componentNames);
title(sprintf('Threshold Detectability (P_d=%.2f)', PdThreshold));
sgtitle([radar.Name ' Link Budget']);

%% Environmental Losses
Fprop = sum(Fprop, 2);
figure('Name', 'Environmental Losses — PMCW Drone Detection');
ax = subplot(4,1,1);
plot(target.ranges*1e-3, Fprop, 'LineWidth', 2);
title('Radar Propagation Factor'); ylabel('(dB)'); grid on;
setYLims(ax, Fprop);

ax = subplot(4,1,2);
plot(target.ranges*1e-3, Lprecipitation, 'LineWidth', 2);
title('Precipitation Loss'); ylabel('(dB)'); grid on;
setYLims(ax, Lprecipitation);

ax = subplot(4,1,3);
plot(target.ranges*1e-3, Llens, 'LineWidth', 2);
title('Lens Effect Loss'); ylabel('(dB)'); grid on;
setYLims(ax, Llens);

ax = subplot(4,1,4);
plot(target.ranges*1e-3, Lgas, 'LineWidth', 2);
title('Atmospheric Gas Loss'); ylabel('(dB)'); grid on;
setYLims(ax, Lgas);
xlabel('Target Range (km)');
sgtitle([radar.Name ' Environmental Losses']);

%% Multi-Elevation SNR Comparison
figure('Name', 'SNR vs Elevation Angle');
elevations = [0 10 30 60];
colors = lines(length(elevations));
hold on;
for k = 1:length(elevations)
    targetK = target;
    targetK.ElevationAngle = elevations(k);
    targetK.Height = el2height(elevations(k), radar.Height, target.ranges, ...
        env.EarthModel, env.EffectiveEarthRadius);
    FpropK = propagationFactor(radar, env, targetK);
    [LgasK, LlensK, LprecipK] = atmosphericLosses(radar, env, targetK);
    LatmosK = sum([LgasK LlensK LprecipK], 2);
    FcustomK = rangeDependentLosses(radar, targetK);
    snrK = radareqsnr(radar.Wavelength, target.ranges, radar.PeakPower, ...
        radar.Pulsewidth, 'Gain', radar.Gain, 'RCS', target.RCS, ...
        'Ts', radar.SystemTemperature, 'PropagationFactor', FpropK, ...
        'AtmosphericLoss', LatmosK, 'CustomFactor', FcustomK);
    plot(target.ranges*1e-3, snrK, 'LineWidth', 2, 'Color', colors(k,:), ...
        'DisplayName', sprintf('El = %d°', elevations(k)));
end
xline(constraint.Range*1e-3, '--k', 'Max Range', 'LineWidth', 1.5);
yline(Dx(2), '--b', 'Objective', 'LineWidth', 1);
yline(Dx(1), '-.r', 'Threshold', 'LineWidth', 1);
hold off;
xlabel('Target Range (km)'); ylabel('SNR (dB)');
title('SNR vs Range at Different Elevation Angles');
legend('Location', 'best'); grid on;

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function radar = designRadar(radarName)
    radar.Name = radarName;

    % --- Scanning ---
    radar.AzScanStartElectronic = -60;
    radar.AzScanStopElectronic  =  60;
    radar.ElScanStartElectronic =   0;
    radar.ElScanStopElectronic  =  30;

    % --- Beamwidths (16×16 array with Taylor taper) ---
    radar.AzimuthBeamwidth   = [7; 7];
    radar.ElevationBeamwidth = [7; 7];

    % --- PMCW Waveform Parameters ---
    radar.Frequency      = 79e9;          % Hz (center of 77-81 GHz)
    radar.PulseBandwidth  = 4e9;           % Hz (chip rate = 4 GHz)
    radar.PeakPower      = 5.74;           % W (= average power for PMCW)
    radar.PRF            = 10e3;           % Hz (code repetition rate)
    % PMCW key: Pulsewidth = 1/PRF (continuous TX, full PRI)
    % This models duty cycle = 1.0, giving +3 dB over pulsed (duty=0.5)
    radar.Pulsewidth     = 1/radar.PRF;    % 100 µs (continuous)

    % --- Hardware ---
    radar.NoiseTemperature = 1500;         % K (~8 dB noise figure)
    radar.ReferenceNoiseTemperature = 290; % K
    radar.Gain = [28.2034, 28.2034];       % dBi (256-element URA, Taylor)

    % --- Detection ---
    radar.NumCoherentPulses    = 512;      % Code repetitions integrated
    radar.NumNonCoherentPulses = 1;
    radar.NumCPIs              = 1;
    radar.Pfa                  = 1e-6;
    radar.CFARMethod           = 'GOCA';   % Greatest-of CFAR
    radar.CFARNumCells         = 32;

    % --- Losses ---
    radar.CustomLoss     = 3;              % dB
    radar.STCCutOffRange = 1200;           % m (1.2 km)
    radar.STCExponent    = 3.5;

    % --- Platform ---
    radar.Height    = 1.7;                 % m (ground-based)
    radar.TiltAngle = 0;                   % deg
    radar.Polarization      = 'H';
    radar.PolarizationAngle = 0;

    % --- Derived ---
    radar.Wavelength = freq2wavelen(radar.Frequency);
    radar.SystemTemperature = radar.NoiseTemperature;
    radar.MaxSpeed = radar.Wavelength * radar.PRF / 4;
    radar.UnambiguousRange = physconst('LightSpeed') / (2 * radar.PRF);
    radar.RangeRateRes = radar.Wavelength / (2 * radar.NumCoherentPulses / radar.PRF);
    radar.EffectiveAzRes = effbeamwidth(radar.AzimuthBeamwidth(1), radar.AzimuthBeamwidth(2));
    radar.EffectiveElRes = effbeamwidth(radar.ElevationBeamwidth(1), radar.ElevationBeamwidth(2));
end

function env = radarEnvironment()
    env.EarthModel = 'Curved';
    env.LatitudeModel = 'Standard';
    env.PrecipitationMinRange = 0;
    env.PrecipitationMaxRange = 1500;        % m (covers operational range)
    env.RainRate = 10;                        % mm/hr (moderate rain)
    env.RainPercentage = 0.01;
    env.SurfaceHeightStandardDeviation = 0;
    env.SurfaceRelativePermittivity = -0 - 1i*1e16;
    env.SurfaceSlope = 0;
    env.VegetationType = 'None';

    [env.RefractiveIndex, env.SurfaceRefractivity] = refractiveidx(0, ...
        'LatitudeModel', env.LatitudeModel);
    env.RefractionExponent = refractionexp(env.SurfaceRefractivity);

    n = refractiveidx([0 1e3], 'LatitudeModel', env.LatitudeModel);
    RGradient = (n(2) - n(1)) / 1e3;
    env.EffectiveEarthRadius = effearthradius(RGradient);
end

function target = radarTarget(radar, env, rangeLimits)
    % CORRECTED: Elevation 0° for worst-case propagation analysis
    target.ElevationAngle = 0;             % deg (horizon, worst case)
    target.RCS = 0.01;                     % m² (small drone, -20 dBsm)
    % CORRECTED: Swerling 1 for realistic fluctuating drone RCS
    target.SwerlingCase = 'Swerling1';

    numTargetPositions = 1000;
    target.ranges = linspace(rangeLimits(1), rangeLimits(2), numTargetPositions).';
    target.validRngIdxs = getValidRangeIdxs(radar, target, env);
    target.Height = el2height(target.ElevationAngle, radar.Height, target.ranges, ...
        env.EarthModel, env.EffectiveEarthRadius);
end

function Fprop = propagationFactor(radar, env, target)
    Fradarprop = -Inf(numel(target.ranges), 2);
    idxGteSurface = target.Height >= env.SurfaceHeightStandardDeviation;
    for ie = 1:2
        Fradarprop(idxGteSurface, ie) = radarpropfactor( ...
            target.ranges(idxGteSurface), radar.Frequency, radar.Height, ...
            target.Height(idxGteSurface), ...
            'Polarization', radar.Polarization, ...
            'SurfaceRelativePermittivity', env.SurfaceRelativePermittivity, ...
            'SurfaceHeightStandardDeviation', env.SurfaceHeightStandardDeviation, ...
            'SurfaceSlope', env.SurfaceSlope, ...
            'VegetationType', env.VegetationType, ...
            'ElevationBeamwidth', radar.ElevationBeamwidth(ie), ...
            'TiltAngle', radar.TiltAngle, ...
            'EffectiveEarthRadius', env.EffectiveEarthRadius, ...
            'RefractiveIndex', env.RefractiveIndex);
    end
    Fradarprop(~idxGteSurface,:) = -inf;
    Fprop = Fradarprop;
end

function [Lgas, Llens, Lprecipitation] = atmosphericLosses(radar, env, target)
    Lgas = zeros(numel(target.ranges), 1);
    idxs = target.validRngIdxs;
    elevationAngles = target.ElevationAngle * ones(size(idxs));
    Lgas(idxs) = tropopl(target.ranges(idxs), radar.Frequency, radar.Height, ...
        elevationAngles, 'LatitudeModel', env.LatitudeModel);

    Llens = zeros(numel(target.ranges), 1);
    Llens(idxs) = lenspl(target.ranges(idxs), radar.Height, elevationAngles, ...
        'LatitudeModel', env.LatitudeModel);

    Lprecipitation = zeros(numel(target.ranges), 1);
    idxPrecip = target.ranges >= env.PrecipitationMinRange & target.validRngIdxs;
    if any(idxPrecip)
        rangeExtentMax = env.PrecipitationMaxRange - env.PrecipitationMinRange;
        rangeExtent = target.ranges(idxPrecip) - env.PrecipitationMinRange;
        rangeExtent = min([rangeExtent, rangeExtentMax*ones(numel(rangeExtent),1)], [], 2);
        Lrain = rainpl(rangeExtent, radar.Frequency, env.RainRate, ...
            target.ElevationAngle, radar.PolarizationAngle, env.RainPercentage);
        Lprecipitation(idxPrecip) = Lrain;
    end
end

function Fcustom = rangeDependentLosses(radar, target)
    Fstc = stcfactor(target.ranges, radar.STCCutOffRange, radar.STCExponent);
    Fcustom = Fstc;
end

function idxs = getValidRangeIdxs(radar, target, env)
    anht = radar.Height + 2*env.SurfaceHeightStandardDeviation;
    tgtht = inf;
    minRng = 0;
    maxRng = 2*env.EffectiveEarthRadius + tgtht + anht;
    idxs = (target.ranges >= minRng & target.ranges <= maxRng);
end

function [Dx, Dc, componentNames, processingGain] = effectiveDetectability(Pd, radar, target)
    Pfa = radar.Pfa;
    [Dc, componentNames] = detectabilityComponents(Pd, Pfa, ...
        radar.NumNonCoherentPulses, target.SwerlingCase, radar);
    Dx = sum(Dc, 2);
    processingGain = -sum(Dc(:,2:end), 2);
end

function [Dc, names] = detectabilityComponents(Pd, Pfa, N, SW, radar)
    D01   = detectability(Pd, Pfa, 1, 'Swerling0');
    Gci   = -pow2db(radar.NumCoherentPulses) * ones(numel(Pd), 1);
    Lcfar = cfarloss(Pfa, radar.CFARNumCells, 'Method', radar.CFARMethod) * ones(numel(Pd), 1);
    Lbeam = beamloss(true) * ones(numel(Pd), 1);

    Larrayscan = 0;
    azScanLimits = [radar.AzScanStartElectronic, radar.AzScanStopElectronic];
    Larrayscan = Larrayscan + arrayscanloss(Pd, Pfa, N, azScanLimits, SW);
    elScanLimits = [radar.ElScanStartElectronic, radar.ElScanStopElectronic];
    Larrayscan = Larrayscan + arrayscanloss(Pd, Pfa, N, elScanLimits, SW);

    Lcustom = radar.CustomLoss * ones(numel(Pd), 1);

    Dc = [D01 Gci Lcfar Lbeam Larrayscan Lcustom];
    names = {'Single-pulse steady target', 'Coherent integration gain', ...
             'CFAR loss', 'Beam shape loss', 'Array scan loss', 'Custom loss'};
end

function setYLims(ax, x)
    idx = isfinite(x);
    minx = min(x(idx), [], 'all');
    if isempty(minx), minx = 0; end
    maxx = max(x(idx), [], 'all');
    if isempty(maxx), maxx = 0; end
    d = max((maxx - minx)/10, 0.1);
    ylim(ax, [minx-d maxx+d]);
end