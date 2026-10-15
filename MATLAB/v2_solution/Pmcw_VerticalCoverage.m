% =========================================================================
% VERTICAL COVERAGE — PMCW Radar for Small Drone Detection
% =========================================================================
% Blake chart showing detection coverage in the vertical plane
% Shows how high/far the radar can detect a small drone (σ = 0.01 m²)
%
% Generated on: 02-Jul-2026 (corrected for PMCW)
% =========================================================================

%% Set up radar and environment
radar = designRadar('DroneDetectionRadar_PMCW');
env   = radarEnvironment();

%% Calculate effective beamwidth
elBwTx = radar.ElevationBeamwidth(1);
elBwRx = radar.ElevationBeamwidth(2);
radar.EffectiveElevationBeamwidth = effbeamwidth(elBwTx, elBwRx);

%% Calculate vertical coverage pattern
rfs    = 1;                        % Range free-space (normalized)
elStep = 0.005729577951308;        % Elevation step size (deg)

[vcpKm, vcpAnglesDeg] = radarvcd(radar.Frequency, rfs, radar.Height, ...
    'Polarization', radar.Polarization, ...
    'SurfaceRelativePermittivity', env.SurfaceRelativePermittivity, ...
    'SurfaceHeightStandardDeviation', env.SurfaceHeightStandardDeviation, ...
    'SurfaceSlope', env.SurfaceSlope, ...
    'VegetationType', env.VegetationType, ...
    'TiltAngle', radar.TiltAngle, ...
    'ElevationBeamwidth', radar.EffectiveElevationBeamwidth, ...
    'EffectiveEarthRadius', env.EffectiveEarthRadius, ...
    'ElevationStepSize', elStep);

%% Plot Blake Chart
figure('Name', [radar.Name ' — Vertical Coverage']);
hAxes = newplot;

blakechart(vcpKm, vcpAnglesDeg, ...
    'ScalePower', 1, ...
    'AntennaHeight', radar.Height*1e-3, ...
    'SurfaceRefractivity', env.SurfaceRefractivity, ...
    'RefractionExponent', env.RefractionExponent, ...
    'Parent', hAxes);
legend(radar.Name);
title('Vertical Coverage — PMCW Drone Detection Radar');

%% Print coverage summary
fprintf('=== Vertical Coverage Summary ===\n');
fprintf('Radar height:         %.1f m\n', radar.Height);
fprintf('Frequency:            %.1f GHz\n', radar.Frequency/1e9);
fprintf('Elevation beamwidth:  %.1f°\n', radar.EffectiveElevationBeamwidth);
fprintf('Elevation scan:       %d° to %d°\n', ...
    radar.ElScanStartElectronic, radar.ElScanStopElectronic);
fprintf('Max VCP range:        %.2f km\n', max(vcpKm));
fprintf('=================================\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function radar = designRadar(radarName)
    radar.Name = radarName;
    radar.AzScanStartElectronic = -60;
    radar.AzScanStopElectronic  =  60;
    radar.ElScanStartElectronic =   0;
    radar.ElScanStopElectronic  =  30;
    radar.AzimuthBeamwidth   = [7; 7];
    radar.ElevationBeamwidth = [7; 7];
    radar.Frequency      = 79e9;
    radar.PulseBandwidth  = 4e9;
    radar.PeakPower      = 5.74;
    radar.PRF            = 10e3;
    radar.Pulsewidth     = 1/10e3;         % PMCW: continuous (100 µs)
    radar.NoiseTemperature = 1500;
    radar.ReferenceNoiseTemperature = 290;
    radar.Gain = [28.2034, 28.2034];
    radar.NumCoherentPulses    = 512;
    radar.NumNonCoherentPulses = 1;
    radar.NumCPIs   = 1;
    radar.Pfa       = 1e-6;
    radar.CFARMethod   = 'GOCA';
    radar.CFARNumCells = 32;
    radar.CustomLoss     = 3;
    radar.STCCutOffRange = 1200;
    radar.STCExponent    = 3.5;
    radar.Height    = 1.7;
    radar.TiltAngle = 0;
    radar.Polarization      = 'H';
    radar.PolarizationAngle = 0;
    radar.Wavelength = freq2wavelen(radar.Frequency);
    radar.SystemTemperature = radar.NoiseTemperature;
    radar.MaxSpeed = radar.Wavelength * radar.PRF / 4;
    radar.UnambiguousRange = physconst('LightSpeed') / (2*radar.PRF);
    radar.RangeRateRes = radar.Wavelength / (2*radar.NumCoherentPulses/radar.PRF);
    radar.EffectiveAzRes = effbeamwidth(radar.AzimuthBeamwidth(1), radar.AzimuthBeamwidth(2));
    radar.EffectiveElRes = effbeamwidth(radar.ElevationBeamwidth(1), radar.ElevationBeamwidth(2));
end

function env = radarEnvironment()
    env.EarthModel = 'Curved';
    env.LatitudeModel = 'Standard';
    env.PrecipitationMinRange = 0;
    env.PrecipitationMaxRange = 1500;
    env.RainRate = 10;
    env.RainPercentage = 0.01;
    env.SurfaceHeightStandardDeviation = 0;
    env.SurfaceRelativePermittivity = -0 - 1i*1e16;
    env.SurfaceSlope = 0;
    env.VegetationType = 'None';
    [env.RefractiveIndex, env.SurfaceRefractivity] = refractiveidx(0, ...
        'LatitudeModel', env.LatitudeModel);
    env.RefractionExponent = refractionexp(env.SurfaceRefractivity);
    n = refractiveidx([0 1e3], 'LatitudeModel', env.LatitudeModel);
    RGradient = (n(2)-n(1))/1e3;
    env.EffectiveEarthRadius = effearthradius(RGradient);
end