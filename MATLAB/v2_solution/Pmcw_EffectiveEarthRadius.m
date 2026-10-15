% =========================================================================
% RADAR DATA GENERATOR — Atmospheric Refracting (Effective Earth Radius)
% =========================================================================
% Simulates PMCW radar detecting a small drone target in a dynamic scenario
% Uses effective Earth radius model for atmospheric refraction
%
% Propagation: 4/3 Earth radius approximation for standard atmosphere
% This models how the radar beam bends due to atmospheric refraction
% by replacing the real Earth radius with an effective (larger) one.
%
% Generated on: 02-Jul-2026 (PMCW drone detection)
% =========================================================================

%% Simulation Parameters
c  = physconst('LightSpeed');
fc = 79e9;                         % Center frequency
BW = 4e9;                          % Chip rate / bandwidth
lambda = c / fc;

%% PMCW Waveform
% Phase-coded waveform (PMCW uses PRBS or similar codes)
chipWidth = 1/BW;                  % 0.25 ns
numChips  = 4096;                  % Code length
codeDuration = numChips * chipWidth; % ~1.024 µs

waveform = phased.PhaseCodedWaveform( ...
    'Code',          'PRBS', ...
    'ChipWidth',     chipWidth, ...
    'NumChips',      numChips, ...
    'SampleRate',    2*BW, ...      % 8 GHz sample rate
    'PRF',           10e3, ...
    'OutputFormat',  'Pulses');

%% Antenna Array (4 subarrays of 8×8 = 256 elements)
f_design = 81e9;
lambda_design = c / f_design;

subarray = phased.URA('Size', [8 8], ...
    'Lattice', 'Rectangular', ...
    'ArrayNormal', 'z', ...
    'ElementSpacing', [lambda_design/2, lambda_design/2]);

% Taylor taper
sll = -30; nbar = 5;
rwind = taylorwin(8, nbar, sll);
cwind = taylorwin(8, nbar, sll);
subarray.Taper = (rwind * cwind.').';

% Sinc element
elem = phased.SincAntennaElement;
elem.FrequencyRange = [77e9 81e9];
elem.Beamwidth = [80 80];
subarray.Element = elem;

% Replicate to 4 subarrays
array = phased.ReplicatedSubarray('Subarray', subarray, ...
    'Layout', 'Rectangular', ...
    'GridSize', [2 2], ...
    'GridSpacing', [8*lambda_design/2, 8*lambda_design/2], ...
    'SubarraySteering', 'Phase', ...
    'PhaseShifterFrequency', f_design);

%% Transmitter and Receiver
peakPower = 5.74;                  % W
txGain = 28.2;                     % dBi

transmitter = phased.Transmitter( ...
    'PeakPower', peakPower, ...
    'Gain', txGain, ...
    'InUseOutputPort', true);

NF = 10*log10(1 + 1500/290);      % ~8 dB from Ts=1500K
receiver = phased.ReceiverPreamp( ...
    'Gain', txGain, ...
    'NoiseFigure', NF, ...
    'SampleRate', 2*BW, ...
    'EnableInputPort', true);

%% Atmospheric Refraction — Effective Earth Radius Model
% Standard atmosphere: effective Earth radius = 4/3 × real Earth radius
% This accounts for how the radar beam curves in the atmosphere
realEarthRadius = 6371e3;          % m
refractionFactor = 4/3;            % Standard atmosphere
effectiveEarthRadius = refractionFactor * realEarthRadius;

fprintf('=== Propagation Model: Effective Earth Radius ===\n');
fprintf('Real Earth radius:      %.0f km\n', realEarthRadius/1e3);
fprintf('Refraction factor:      %.4f\n', refractionFactor);
fprintf('Effective Earth radius: %.0f km\n', effectiveEarthRadius/1e3);

%% Free-Space Propagation Channel
channel = phased.FreeSpace( ...
    'SampleRate', 2*BW, ...
    'PropagationSpeed', c, ...
    'OperatingFrequency', fc, ...
    'TwoWayPropagation', true);

%% Target (Small Drone)
targetRCS = 0.01;                  % m² (-20 dBsm)
target = phased.RadarTarget( ...
    'MeanRCS', targetRCS, ...
    'Model', 'Swerling1', ...
    'OperatingFrequency', fc);

%% Scenario Definition
% Radar platform (ground-based, stationary)
radarPos = [0; 0; 1.7];           % 1.7 m height
radarVel = [0; 0; 0];

% Drone target (approaching at 50 m/s, 100 m altitude, 800 m initial range)
droneInitPos  = [800; 0; 100];
droneVel      = [-50; 10; 0];     % Approaching + lateral movement

% Simulation time
PRF = 10e3;
numPulses = 512;                   % Coherent processing interval
PRI = 1/PRF;
simTime = numPulses * PRI;         % 51.2 ms

fprintf('\n=== Simulation Parameters ===\n');
fprintf('Drone initial position: [%.0f, %.0f, %.0f] m\n', droneInitPos);
fprintf('Drone velocity:         [%.0f, %.0f, %.0f] m/s\n', droneVel);
fprintf('Drone RCS:              %.4f m² (%.0f dBsm)\n', targetRCS, 10*log10(targetRCS));
fprintf('Number of pulses:       %d\n', numPulses);
fprintf('CPI duration:           %.1f ms\n', simTime*1e3);

%% Run Simulation
% Pre-allocate received signal cube
numSamples = round(2*BW / PRF);
rxSignalCube = complex(zeros(numSamples, numPulses));

dronePos = droneInitPos;

for pulse = 1:numPulses
    % Update drone position
    dronePos = droneInitPos + droneVel * (pulse-1) * PRI;

    % Apply effective Earth radius correction to target height
    slantRange = norm(dronePos - radarPos);
    groundRange = sqrt((dronePos(1)-radarPos(1))^2 + (dronePos(2)-radarPos(2))^2);
    heightCorrection = groundRange^2 / (2 * effectiveEarthRadius);
    effectiveTargetHeight = dronePos(3) - heightCorrection;

    % Check if target is above horizon
    if effectiveTargetHeight < 0
        fprintf('Pulse %d: Target below horizon (effective height = %.1f m)\n', ...
            pulse, effectiveTargetHeight);
        continue;
    end

    % Generate PMCW waveform
    txWaveform = waveform();

    % Transmit
    [txSig, txStatus] = transmitter(txWaveform);

    % Propagate (two-way)
    rxSig = channel(txSig, radarPos, dronePos, radarVel, droneVel);

    % Target reflection
    rxSig = target(rxSig);

    % Receive
    rxSig = receiver(rxSig, ~txStatus);

    % Store
    if length(rxSig) >= numSamples
        rxSignalCube(:, pulse) = rxSig(1:numSamples);
    end
end

%% Matched Filter Processing
matchedFilter = phased.MatchedFilter( ...
    'Coefficients', getMatchedFilter(waveform));
rxCompressed = matchedFilter(rxSignalCube);

%% Range-Doppler Map
rdMap = fftshift(fft(rxCompressed, [], 2), 2);
rdMap_dB = 20*log10(abs(rdMap) + eps);

% Axes
rangeAxis = (0:numSamples-1) * c / (2 * 2*BW);
dopplerAxis = linspace(-PRF/2, PRF/2, numPulses);
velocityAxis = dopplerAxis * lambda / 2;

%% Plot Range-Doppler Map
figure('Name', 'Range-Doppler Map — Effective Earth Radius');
imagesc(velocityAxis, rangeAxis, rdMap_dB);
axis xy;
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title(sprintf('Range-Doppler Map — PMCW (\\sigma=%.2f m², Eff. Earth Radius)', targetRCS));
colorbar;
colormap(jet);
caxis([max(rdMap_dB(:))-60, max(rdMap_dB(:))]);

%% Plot Range Profile (integrated over Doppler)
figure('Name', 'Range Profile — Effective Earth Radius');
rangeProfile = max(rdMap_dB, [], 2);
plot(rangeAxis, rangeProfile, 'LineWidth', 1.5);
xlabel('Range (m)');
ylabel('Amplitude (dB)');
title('Range Profile — PMCW with Effective Earth Radius');
grid on;

% Mark expected target range
expectedRange = norm(droneInitPos - radarPos);
xline(expectedRange, '--r', sprintf('Target @ %.0f m', expectedRange), 'LineWidth', 1.5);

fprintf('\n=== Results ===\n');
fprintf('Expected target range:  %.1f m\n', expectedRange);
[~, peakIdx] = max(rangeProfile);
fprintf('Detected peak range:    %.1f m\n', rangeAxis(peakIdx));
fprintf('Peak SNR:               %.1f dB\n', max(rangeProfile) - median(rangeProfile));
fprintf('================================\n');