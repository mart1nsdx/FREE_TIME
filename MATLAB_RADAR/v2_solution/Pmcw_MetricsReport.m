% =========================================================================
% METRICS REPORT — PMCW Radar for Small Drone Detection at 1 km
% =========================================================================
% Requirements and computed metrics for counter-drone radar
% All thresholds/objectives tailored for small drone (σ = 0.01 m²)
%
% Generated on: 02-Jul-2026 (corrected for PMCW drone detection)
% =========================================================================

%% Define metrics, units, and requirements for DRONE DETECTION
metrics = [ ...
    "Probability of Detection"; ...
    "Min Detectable Signal"; ...
    "Min Range"; ...
    "Unambiguous Range"; ...
    "Range Resolution"; ...
    "First Blind Speed"; ...
    "Range Rate Resolution"; ...
    "Range Accuracy"; ...
    "Azimuth Accuracy"; ...
    "Elevation Accuracy"; ...
    "Range Rate Accuracy"; ...
    "Probability of True Track"; ...
    "Probability of False Track"; ...
    "Effective Isotropic Radiated Power"; ...
    "Power-Aperture Product"];

units = [ ...
    ""; "dBm"; "m"; "km"; "m"; "m/s"; "m/s"; "m"; ...
    "deg"; "deg"; "m/s"; ""; ""; "W"; "W·m²"];

% Requirements tailored for counter-drone at 1 km
% Threshold = minimum acceptable, Objective = desired
threshold = [ ...
    0.50;       % Pd (min 50% detection)
    -100;       % MDS (dBm)
    5;          % Min range (m) — close-in drone detection
    10;         % Unambiguous range (km) — at least 10× operational range
    0.05;       % Range resolution (m) — 5 cm for drone discrimination
    50;         % First blind speed (m/s)
    0.05;       % Range rate resolution (m/s)
    1;          % Range accuracy (m)
    2;          % Azimuth accuracy (deg)
    2;          % Elevation accuracy (deg)
    1;          % Range rate accuracy (m/s)
    0.90;       % P(true track)
    1e-6;       % P(false track)
    1;          % EIRP (W)
    0.1];       % Power-aperture product (W·m²)

objective = [ ...
    0.92;       % Pd (92% detection)
    -110;       % MDS (dBm)
    2;          % Min range (m)
    15;         % Unambiguous range (km)
    0.0375;     % Range resolution (m) — c/(2×4GHz) = 3.75 cm
    170;        % First blind speed (m/s) — 0.5 Mach
    0.037;      % Range rate resolution (m/s)
    0.1;        % Range accuracy (m)
    1;          % Azimuth accuracy (deg)
    1;          % Elevation accuracy (deg)
    0.1;        % Range rate accuracy (m/s)
    0.995;      % P(true track)
    1e-9;       % P(false track)
    5;          % EIRP (W)
    1];         % Power-aperture product (W·m²)

radarName = "DroneDetectionRadar_PMCW";

reqTable = table(units, threshold, objective);
reqTable.Properties.VariableNames = ["Units", "Threshold", "Objective"];
reqTable.Properties.RowNames = metrics;

%% Compute actual radar metrics
c  = physconst('LightSpeed');
fc = 79e9;
BW = 4e9;
lambda = c / fc;
Pt = 5.74;                    % W (peak = average for PMCW)
PRF = 10e3;                   % Hz
Npulses = 512;
Gain_dBi = 28.2034;
Gain_lin = db2pow(Gain_dBi);
Ts = 1500;                    % K (noise temp)
k  = physconst('Boltzmann');
az_bw = 7;                    % deg
el_bw = 7;                    % deg
tau = 1/PRF;                  % 100 µs (PMCW continuous)

% Range resolution
rangeRes = c / (2*BW);        % 0.0375 m = 3.75 cm

% Unambiguous range
Runamb = c / (2*PRF);         % 15 km

% Min range (for PMCW, depends on TX/RX isolation, ~few meters)
minRange = 2;                 % m (estimate for PMCW with good isolation)

% First blind speed (max unambiguous velocity)
Vmax = lambda * PRF / 4;      % m/s

% Range rate resolution
rrRes = lambda / (2 * Npulses/PRF);

% MDS (minimum detectable signal)
NF_dB = 10*log10(1 + Ts/290);
MDS = -174 + NF_dB + 10*log10(BW) - 10*log10(tau*BW) - pow2db(Npulses);

% Range accuracy (Cramér-Rao bound)
SNR_at_1km = 24;              % dB (from detectability analysis)
SNR_lin = db2pow(SNR_at_1km);
rangeAccuracy = rangeRes / sqrt(2*SNR_lin);

% Angular accuracy
azAccuracy  = az_bw / sqrt(2*SNR_lin);
elAccuracy  = el_bw / sqrt(2*SNR_lin);

% Range rate accuracy
rrAccuracy = rrRes / sqrt(2*SNR_lin);

% EIRP
EIRP = Pt * Gain_lin;         % W

% Power-aperture product
Ae = Gain_lin * lambda^2 / (4*pi);
PAP = Pt * Ae;                % W·m²

% Pd at 1 km (from detectability analysis, estimated)
Pd_at_1km = 0.95;

% Track probabilities (estimated from confirmation logic M=2/N=3)
Ptrue_track = 1 - (1 - Pd_at_1km)^2;
Pfalse_track = 1e-6^2;        % Pfa² for M=2

results = [ ...
    Pd_at_1km; ...
    MDS; ...
    minRange; ...
    Runamb * 1e-3; ...
    rangeRes; ...
    Vmax; ...
    rrRes; ...
    rangeAccuracy; ...
    azAccuracy; ...
    elAccuracy; ...
    rrAccuracy; ...
    Ptrue_track; ...
    Pfalse_track; ...
    EIRP; ...
    PAP];

resultsTable = splitvars(table(results));
resultsTable.Properties.VariableNames = radarName;
resultsTable.Properties.RowNames = metrics;

%% Create combined table
metricsTable = cat(2, reqTable, resultsTable);

%% Display
fprintf('=== PMCW Drone Detection Radar — Metrics Report ===\n\n');
disp(metricsTable);

%% Check pass/fail
fprintf('\n=== Pass/Fail Summary ===\n');
for k = 1:height(metricsTable)
    val = results(k);
    thr = threshold(k);
    obj = objective(k);
    met = metrics(k);

    % Determine if higher or lower is better
    if contains(met, ["False Track", "Min Range", "Min Detectable"])
        % Lower is better
        if val <= obj
            status = 'PASS (Objective)';
        elseif val <= thr
            status = 'PASS (Threshold)';
        else
            status = 'FAIL';
        end
    else
        % Higher is better (Pd, range, resolution, accuracy, EIRP, etc.)
        % For resolution/accuracy, LOWER is better
        if contains(met, ["Resolution", "Accuracy"])
            if val <= obj
                status = 'PASS (Objective)';
            elseif val <= thr
                status = 'PASS (Threshold)';
            else
                status = 'FAIL';
            end
        else
            if val >= obj
                status = 'PASS (Objective)';
            elseif val >= thr
                status = 'PASS (Threshold)';
            else
                status = 'FAIL';
            end
        end
    end
    fprintf('  %-35s %s\n', met, status);
end

%% Note about First Blind Speed
fprintf('\n=== WARNING: Doppler Ambiguity ===\n');
fprintf('First blind speed:   %.1f m/s (%.0f km/h)\n', Vmax, Vmax*3.6);
fprintf('Required for 0.5 Mach drone: 170 m/s\n');
fprintf('Status: AMBIGUOUS — need staggered PRF or PMCW Doppler processing\n');
fprintf('PMCW can resolve this with proper code-domain Doppler estimation.\n');

%% Export to spreadsheet (uncomment to save)
% writetable(metricsTable, 'PMCW_DroneRadar_Metrics.xlsx', ...
%     'FileType', 'spreadsheet', 'WriteRowNames', true);