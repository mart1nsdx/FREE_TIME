%% PMCW/FMCW 79 GHz Canted Parallel-Fed Array Design
% Serious preliminary antenna-array design for a 64-Tx / 64-Rx physical-channel
% radar front-end inspired by the AWR2943/AWR2944 per-channel RF power level.
%
% HOW TO USE
% 1) Put this file in the same folder as RADAR_BUENO.mat and your PMCW radar
%    detectability script if you want the optional import section to run.
% 2) Open this file in MATLAB. It is sectioned for the Live Editor.
% 3) Run all sections. It will produce:
%      - patch and feed dimensions at 79 GHz
%      - 64-channel Tx and 64-channel Rx aperture geometry
%      - 1024 Tx radiating patches and 1024 Rx radiating patches
%      - 4 canted subpanels inside a 150 mm x 150 mm mechanical envelope
%      - fixed corporate-feed amplitude taper / element weighting tables
%      - broadside and steered array-factor simulations
%      - 3-D aperture visualization
%      - radarAntennaUpdate_79GHz.mat for integration with the radar code
%
% IMPORTANT ENGINEERING NOTE
% This is a high-quality first-pass synthesis and array-factor simulation. It is
% NOT a final manufacturable 79 GHz layout. A final design must be validated in a
% full-wave EM solver such as Antenna Toolbox MoM for small blocks, HFSS, CST, or
% FEKO, followed by stack-up/tolerance/connector/thermal measurements.

clear; close all; clc;
format compact;

%% 0. Optional import of RADAR_BUENO.mat
% The uploaded radar file appears to be a Radar Toolbox export. This section is
% defensive: the antenna design below is self-contained, but if RADAR_BUENO.mat
% exists, the script loads it and reports the available fields.
radarMatFile = 'RADAR_BUENO.mat';
radarData = struct();
if isfile(radarMatFile)
    tmp = load(radarMatFile);
    radarData = tmp;
    fprintf('Loaded %s. Top-level variables:\n', radarMatFile);
    disp(fieldnames(tmp));
else
    fprintf('No RADAR_BUENO.mat found in the current folder. Continuing with explicit design values.\n');
end

%% 1. Radar, mechanical, and channel requirements
% Frequency is moved to the intended 76-81 GHz automotive/mmWave band. The
% detectability code you supplied used a PMCW-equivalent pulsed framework; this
% antenna design supplies the physical aperture gain, power, beamwidth, and scan
% values that should replace the simplified sinc antenna numbers.

req = struct();
req.f0                 = 79e9;           % Hz, center design frequency
req.band               = [76e9 81e9];    % Hz, chip operating band envelope
req.c                  = physconst_local('LightSpeed');
req.lambda0            = req.c/req.f0;
req.maxAreaXY          = [150e-3 150e-3]; % m x m projected mechanical envelope
req.maxPanelTiltDeg    = 10;             % deg, maximum allowed mechanical cant
req.numTxChannels      = 64;
req.numRxChannels      = 64;
req.txPowerPerChannel_dBm = 13.5;        % dBm, reference per physical Tx chain
req.txPowerPerChannel_W   = 1e-3*10^(req.txPowerPerChannel_dBm/10);
req.txTotalPower_W        = req.numTxChannels*req.txPowerPerChannel_W;
req.txTotalPower_dBm      = 10*log10(req.txTotalPower_W/1e-3);

% Receiver NF in the chip image is 12 dB. Keep this separate from the antenna
% design; it matters when updating the radar link budget.
req.rxNoiseFigure_dB = 12;
req.T0 = 290;
req.receiverSystemTemperature_K = req.T0*10^(req.rxNoiseFigure_dB/10);

fprintf('\n===== Requirement summary =====\n');
fprintf('f0                         = %.2f GHz\n', req.f0/1e9);
fprintf('lambda0                    = %.3f mm\n', req.lambda0*1e3);
fprintf('physical Tx channels       = %d\n', req.numTxChannels);
fprintf('physical Rx channels       = %d\n', req.numRxChannels);
fprintf('Tx power/channel           = %.2f dBm = %.2f mW\n', req.txPowerPerChannel_dBm, req.txPowerPerChannel_W*1e3);
fprintf('Total coherent Tx power    = %.2f W = %.2f dBm\n', req.txTotalPower_W, req.txTotalPower_dBm);
fprintf('Receiver NF assumption     = %.1f dB -> Tsys approx %.0f K\n', req.rxNoiseFigure_dB, req.receiverSystemTemperature_K);
fprintf('Mechanical envelope        = %.0f mm x %.0f mm\n', req.maxAreaXY(1)*1e3, req.maxAreaXY(2)*1e3);
fprintf('Max panel cant             = %.1f deg\n', req.maxPanelTiltDeg);

%% 2. Substrate and single inset-fed rectangular patch synthesis
% A low-loss mmWave laminate is assumed. Replace the values with the exact
% fabrication stack-up when known. 5 mil / 0.127 mm RT/duroid 5880-class laminate
% is a reasonable starting point at 79 GHz because it reduces surface-wave loss
% and keeps feed widths manufacturable.

sub = struct();
sub.name       = 'Rogers RT/duroid 5880 class';
sub.er         = 2.20;
sub.tand       = 9e-4;
sub.h          = 0.127e-3;       % m, 5 mil
sub.copperT    = 17e-6;          % m, 1/2 oz copper approximate
sub.minTrace   = 75e-6;          % m, conservative mmWave PCB rule-of-thumb
sub.minGap     = 75e-6;          % m

patch = synthesizeRectPatch(req.f0, sub.er, sub.h);
patch.groundMargin = 0.45*req.lambda0;  % local ground around one patch for element study
patch.groundW      = patch.W + 2*patch.groundMargin;
patch.groundL      = patch.L + 2*patch.groundMargin;

% Feed-line dimensions for the assumed stack-up.
lineZ = [25 35.35 50 70.71 86.6 100 120 150];
lineW = zeros(size(lineZ));
for k = 1:numel(lineZ)
    lineW(k) = microstripWidthFromZ0(lineZ(k), sub.h, sub.er);
end
lineTable = table(lineZ(:), lineW(:)*1e3, (req.lambda0/sqrt(patch.eeff)/4)*ones(numel(lineZ),1)*1e3, ...
    'VariableNames', {'Z0_ohm','Width_mm','QuarterWaveLength_mm'});

fprintf('\n===== Single patch synthesis =====\n');
fprintf('Substrate                  = %s\n', sub.name);
fprintf('er, tanD, h                = %.3f, %.4g, %.3f mm\n', sub.er, sub.tand, sub.h*1e3);
fprintf('Patch W x L                = %.3f mm x %.3f mm\n', patch.W*1e3, patch.L*1e3);
fprintf('Effective permittivity     = %.3f\n', patch.eeff);
fprintf('Delta-L fringe correction  = %.3f mm per side\n', patch.deltaL*1e3);
fprintf('Guided wavelength approx   = %.3f mm\n', patch.lambdaG*1e3);
fprintf('Inset feed offset y0       = %.3f mm from radiating edge for Rin approx 50 ohm\n', patch.insetY0*1e3);
fprintf('50 ohm microstrip width    = %.3f mm\n', lineW(lineZ==50)*1e3);
fprintf('\nMicrostrip line table:\n');
disp(lineTable);

%% 3. Array architecture decision
% The requested architecture is interpreted as:
%   - 64 independent Tx RF channels
%   - 64 independent Rx RF channels
%   - each RF channel feeds a fixed passive 4 x 4 parallel-fed weighted patch tile
%   - four mechanical panels; each panel carries 16 Tx tiles + 16 Rx tiles
%   - total Tx radiators = 64 x 16 = 1024 patches
%   - total Rx radiators = 64 x 16 = 1024 patches
%
% This is the best compromise between gain and beamforming realism. Making one
% RF channel feed more than 4 x 4 patches increases gain but degrades electronic
% scan because the channel phase centers become too sparse. A 4 x 4 passive tile
% places the first tile-factor null close to the first super-array grating lobe,
% so broadside and +/-10 deg steering remain usable.

cfg = struct();
cfg.panelCountXY       = [2 2];
cfg.numPanels          = prod(cfg.panelCountXY);
cfg.panelSide          = 75e-3;           % m, 2 x 2 panels fill 150 mm x 150 mm projection
cfg.panelTiltDeg       = 10;              % deg, pyramid/canted arrangement
cfg.elementPitch       = req.lambda0/2;   % m, free-space half-wavelength pitch
cfg.tileSize           = [4 4];           % passive radiators per RF channel
cfg.channelGridPerPanel= [4 4];           % 16 physical Tx channels per panel, 16 Rx per panel
cfg.radiatorsPerPanelXY= cfg.tileSize.*cfg.channelGridPerPanel; % 16 x 16 per Tx block and Rx block
cfg.radiatorsPerTile   = prod(cfg.tileSize);
cfg.txChannelsPerPanel = prod(cfg.channelGridPerPanel);
cfg.rxChannelsPerPanel = prod(cfg.channelGridPerPanel);
cfg.totalTxRadiators   = cfg.numPanels*cfg.txChannelsPerPanel*cfg.radiatorsPerTile;
cfg.totalRxRadiators   = cfg.numPanels*cfg.rxChannelsPerPanel*cfg.radiatorsPerTile;
cfg.txRxBlockOffset    = 18e-3;           % m, local separation between Tx and Rx aperture blocks on each panel
cfg.feedNetworkLoss_dB = 1.8;             % dB, corporate feed + transitions + copper/dielectric preliminary
cfg.ebgIsolation_dB_goal = 15;            % dB target improvement to Tx/Rx and tile-to-tile isolation, to be EM-verified

% Weighting. SLL target is a compromise: deep taper lowers side lobes but costs
% power/aperture efficiency. 25 dB is a robust first choice for tracking radar.
cfg.sllTarget_dB = -25;
cfg.tileTaper    = normalizedTaper2D(cfg.tileSize(1), cfg.tileSize(2), cfg.sllTarget_dB);
cfg.channelTaper = normalizedTaper2D(cfg.channelGridPerPanel(1), cfg.channelGridPerPanel(2), cfg.sllTarget_dB);
cfg.panelTaper   = normalizedTaper2D(2, 2, -18); % gentle 2x2 panel taper

fprintf('\n===== Array architecture =====\n');
fprintf('Panels                       = %d, each %.1f mm x %.1f mm, canted %.1f deg\n', cfg.numPanels, cfg.panelSide*1e3, cfg.panelSide*1e3, cfg.panelTiltDeg);
fprintf('Element pitch                = %.3f mm = %.3f lambda0\n', cfg.elementPitch*1e3, cfg.elementPitch/req.lambda0);
fprintf('RF channel tile              = %d x %d passive patches\n', cfg.tileSize(1), cfg.tileSize(2));
fprintf('RF channel grid per panel    = %d x %d Tx and %d x %d Rx\n', cfg.channelGridPerPanel(1), cfg.channelGridPerPanel(2), cfg.channelGridPerPanel(1), cfg.channelGridPerPanel(2));
fprintf('Total Tx radiating patches   = %d\n', cfg.totalTxRadiators);
fprintf('Total Rx radiating patches   = %d\n', cfg.totalRxRadiators);
fprintf('Tx/Rx block offset per panel = %.1f mm\n', cfg.txRxBlockOffset*1e3);

%% 4. Metamaterial / EBG cell first-pass dimensions
% The metamaterial part is implemented as an EBG/mushroom fence and local
% high-impedance surface around tiles and between Tx/Rx blocks. It should be
% swept in a full-wave solver; the values below are intentionally conservative
% and manufacturable for high-end mmWave PCB processes.

ebg = struct();
ebg.pitch       = 0.55e-3;   % m, about lambda0/6.9
ebg.patchSize   = 0.45e-3;   % m
ebg.gap         = ebg.pitch - ebg.patchSize;
ebg.viaDiameter = 0.10e-3;   % m, laser/mechanical microvia class
ebg.ringsBetweenTxRx = 3;
ebg.ringsAroundTiles = 1;
ebg.note = 'First-pass EBG fence; tune by eigenmode/unit-cell and full aperture EM simulation.';

% Crude LC estimate for a mushroom EBG cell. Use only as a sanity check; do not
% treat it as a final bandgap prediction.
mu0 = 4*pi*1e-7;
eps0 = 8.854187817e-12;
Lvia = mu0*sub.h;
Cgap = eps0*(sub.er+1)/2 * ebg.patchSize/pi * acosh((ebg.pitch)/(max(ebg.gap,1e-9)));
ebg.fLC_est = 1/(2*pi*sqrt(Lvia*Cgap));

fprintf('\n===== EBG / metamaterial first pass =====\n');
fprintf('EBG pitch / patch / gap       = %.3f / %.3f / %.3f mm\n', ebg.pitch*1e3, ebg.patchSize*1e3, ebg.gap*1e3);
fprintf('EBG via diameter              = %.3f mm\n', ebg.viaDiameter*1e3);
fprintf('Crude LC resonance estimate   = %.1f GHz (must be EM tuned near 76-81 GHz bandgap)\n', ebg.fLC_est/1e9);
fprintf('%s\n', ebg.note);

%% 5. Corporate parallel-feed and fixed element weighting
% Element weighting is implemented as fixed unequal power division inside each
% passive tile and optionally per channel. Digital/RF beam steering occurs at the
% 64 physical Tx and 64 physical Rx channel phase shifters. The fixed passive
% taper reduces side lobes and tile grating artifacts.

feed = struct();
feed.Zref = 50;
feed.tilePower = abs(cfg.tileTaper).^2;
feed.tilePower = feed.tilePower/sum(feed.tilePower(:));
feed.channelPower = abs(cfg.channelTaper).^2;
feed.channelPower = feed.channelPower/sum(feed.channelPower(:));
feed.panelPower = abs(cfg.panelTaper).^2;
feed.panelPower = feed.panelPower/sum(feed.panelPower(:));

% Ideal terminal branch impedances if a star divider fed all branches directly.
% A real layout should implement this as cascaded unequal Wilkinson dividers and
% quarter-wave impedance transformers bounded to manufacturable line impedances.
feed.tileIdealTerminalZ = feed.Zref ./ feed.tilePower;
feed.channelIdealTerminalZ = feed.Zref ./ feed.channelPower;

% Hierarchical split tables for a 4x4 tile.
feed.tileSplitTable = corporateSplitTable2D(feed.tilePower, feed.Zref, 'tile');
feed.channelSplitTable = corporateSplitTable2D(feed.channelPower, feed.Zref, 'channel');

fprintf('\n===== Passive corporate feed / taper =====\n');
fprintf('Tile amplitude taper, normalized RMS power:\n');
disp(cfg.tileTaper);
fprintf('Tile power fractions:\n');
disp(feed.tilePower);
fprintf('Corporate split table for the 4x4 patch tile:\n');
disp(feed.tileSplitTable);
warnHighZ(feed.tileSplitTable, 120, 20);

%% 6. Build the full 3-D canted aperture geometry
geom = buildCantedArrayGeometry(req, cfg, patch);

fprintf('\n===== Geometry check =====\n');
fprintf('Tx elements generated = %d\n', size(geom.tx.pos,1));
fprintf('Rx elements generated = %d\n', size(geom.rx.pos,1));
minXYZ = min([geom.tx.pos; geom.rx.pos], [], 1);
maxXYZ = max([geom.tx.pos; geom.rx.pos], [], 1);
fprintf('Projected X span      = %.2f mm\n', (maxXYZ(1)-minXYZ(1))*1e3);
fprintf('Projected Y span      = %.2f mm\n', (maxXYZ(2)-minXYZ(2))*1e3);
fprintf('Z height span due cant= %.2f mm\n', (maxXYZ(3)-minXYZ(3))*1e3);

if (maxXYZ(1)-minXYZ(1)) > req.maxAreaXY(1) || (maxXYZ(2)-minXYZ(2)) > req.maxAreaXY(2)
    warning('Projected aperture exceeds the specified 150 mm x 150 mm envelope. Reduce offsets or panel side.');
end

%% 7. Plot the 3-D physical aperture
fig3d = figure('Name','79 GHz 64Tx/64Rx canted metamaterial parallel-fed aperture','Color','w','Position',[50 80 1200 800]);
plotCantedArray3D(geom, cfg, patch, ebg);
title('79 GHz canted 4-panel aperture: 64 Tx channels + 64 Rx channels, each feeding a 4x4 weighted patch tile');

%% 8. Array-factor and beamforming simulation
% The pattern below is an array-factor simulation with an approximate patch
% broadside element factor. It is suitable for aperture sizing, beamwidth, side
% lobe, grating-lobe, and steering decisions. It is not a substitute for full EM.

sim = struct();
sim.thetaDeg = -70:0.05:70;      % angular cut about mechanical boresight
sim.elemPatternExponent = 1.6;    % cos(theta)^q approximate patch broadside pattern
sim.scanCases = [0 0; 10 0; -10 0; 0 10; 0 -10]; % [thetaX thetaY] deg from boresight

% Complex weights include passive tapers. Steering phases are applied in the
% arrayFactorCut function per scan case.
txW = geom.tx.weights;
rxW = geom.rx.weights;

patternResults = struct();
for sIdx = 1:size(sim.scanCases,1)
    scanX = sim.scanCases(sIdx,1);
    scanY = sim.scanCases(sIdx,2);
    [patTxAz_dB, theta] = arrayFactorCut(geom.tx.pos, geom.tx.normal, txW, req.f0, sim.thetaDeg, 'az', scanX, scanY, sim.elemPatternExponent);
    [patTxEl_dB, ~]     = arrayFactorCut(geom.tx.pos, geom.tx.normal, txW, req.f0, sim.thetaDeg, 'el', scanX, scanY, sim.elemPatternExponent);
    [patRxAz_dB, ~]     = arrayFactorCut(geom.rx.pos, geom.rx.normal, rxW, req.f0, sim.thetaDeg, 'az', scanX, scanY, sim.elemPatternExponent);
    [patRxEl_dB, ~]     = arrayFactorCut(geom.rx.pos, geom.rx.normal, rxW, req.f0, sim.thetaDeg, 'el', scanX, scanY, sim.elemPatternExponent);

    patternResults(sIdx).scan = [scanX scanY]; %#ok<SAGROW>
    patternResults(sIdx).theta = theta;
    patternResults(sIdx).txAz = patTxAz_dB;
    patternResults(sIdx).txEl = patTxEl_dB;
    patternResults(sIdx).rxAz = patRxAz_dB;
    patternResults(sIdx).rxEl = patRxEl_dB;
    patternResults(sIdx).txAzHPBW = hpbw(theta, patTxAz_dB);
    patternResults(sIdx).txElHPBW = hpbw(theta, patTxEl_dB);
    patternResults(sIdx).rxAzHPBW = hpbw(theta, patRxAz_dB);
    patternResults(sIdx).rxElHPBW = hpbw(theta, patRxEl_dB);
    patternResults(sIdx).txAzSLL  = estimateSLL(theta, patTxAz_dB);
    patternResults(sIdx).txElSLL  = estimateSLL(theta, patTxEl_dB);
    patternResults(sIdx).rxAzSLL  = estimateSLL(theta, patRxAz_dB);
    patternResults(sIdx).rxElSLL  = estimateSLL(theta, patRxEl_dB);
end

figPat = figure('Name','Array-factor cuts, Tx aperture','Color','w','Position',[80 80 1100 760]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; hold on; grid on;
for sIdx = 1:numel(patternResults)
    plot(patternResults(sIdx).theta, patternResults(sIdx).txAz, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('scan X=%+.0f, Y=%+.0f deg', patternResults(sIdx).scan(1), patternResults(sIdx).scan(2)));
end
xlabel('Angle from boresight in azimuth cut, theta_X (deg)'); ylabel('Normalized gain (dB)'); ylim([-60 2]);
title('Tx azimuth cut'); legend('Location','southoutside','Orientation','horizontal');
nexttile; hold on; grid on;
for sIdx = 1:numel(patternResults)
    plot(patternResults(sIdx).theta, patternResults(sIdx).txEl, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('scan X=%+.0f, Y=%+.0f deg', patternResults(sIdx).scan(1), patternResults(sIdx).scan(2)));
end
xlabel('Angle from boresight in elevation cut, theta_Y (deg)'); ylabel('Normalized gain (dB)'); ylim([-60 2]);
title('Tx elevation cut'); legend('Location','southoutside','Orientation','horizontal');

figRx = figure('Name','Array-factor cuts, Rx aperture','Color','w','Position',[120 120 1100 760]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; hold on; grid on;
for sIdx = 1:numel(patternResults)
    plot(patternResults(sIdx).theta, patternResults(sIdx).rxAz, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('scan X=%+.0f, Y=%+.0f deg', patternResults(sIdx).scan(1), patternResults(sIdx).scan(2)));
end
xlabel('Angle from boresight in azimuth cut, theta_X (deg)'); ylabel('Normalized gain (dB)'); ylim([-60 2]);
title('Rx azimuth cut'); legend('Location','southoutside','Orientation','horizontal');
nexttile; hold on; grid on;
for sIdx = 1:numel(patternResults)
    plot(patternResults(sIdx).theta, patternResults(sIdx).rxEl, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('scan X=%+.0f, Y=%+.0f deg', patternResults(sIdx).scan(1), patternResults(sIdx).scan(2)));
end
xlabel('Angle from boresight in elevation cut, theta_Y (deg)'); ylabel('Normalized gain (dB)'); ylim([-60 2]);
title('Rx elevation cut'); legend('Location','southoutside','Orientation','horizontal');

summaryRows = [];
for sIdx = 1:numel(patternResults)
    summaryRows = [summaryRows; ... %#ok<AGROW>
        patternResults(sIdx).scan, patternResults(sIdx).txAzHPBW, patternResults(sIdx).txElHPBW, ...
        patternResults(sIdx).rxAzHPBW, patternResults(sIdx).rxElHPBW, ...
        patternResults(sIdx).txAzSLL, patternResults(sIdx).txElSLL, patternResults(sIdx).rxAzSLL, patternResults(sIdx).rxElSLL];
end
beamSummary = array2table(summaryRows, 'VariableNames', ...
    {'ScanX_deg','ScanY_deg','TxAzHPBW_deg','TxElHPBW_deg','RxAzHPBW_deg','RxElHPBW_deg', ...
     'TxAzSLL_dB','TxElSLL_dB','RxAzSLL_dB','RxElSLL_dB'});

fprintf('\n===== Array-factor beam summary =====\n');
disp(beamSummary);

%% 9. Approximate absolute gain, EIRP, and radar-code update values
% Two gain estimates are given:
%   A) element-count estimate: Gpatch + 10log10(N) - taper loss - feed loss
%   B) aperture bound: 4*pi*Aeff/lambda^2 with aperture efficiency eta
% The conservative design value is min(A,B).

perf = struct();
perf.singlePatchGain_dBi = 6.5;           % typical rectangular patch broadside gain, refine by EM
perf.txTaperLoss_dB = taperLossdB(txW);
perf.rxTaperLoss_dB = taperLossdB(rxW);
perf.feedLoss_dB = cfg.feedNetworkLoss_dB;
perf.apertureEfficiency = 0.55;

% Effective projected area for Tx and Rx blocks. This is conservative because
% the four separated panels also create a larger interferometric aperture.
activeBlockSpan = (cfg.radiatorsPerPanelXY(1)-1)*cfg.elementPitch + patch.W;
activeBlockArea = activeBlockSpan^2;
perf.txActiveArea_m2 = cfg.numPanels*activeBlockArea;
perf.rxActiveArea_m2 = cfg.numPanels*activeBlockArea;

perf.txGain_count_dBi = perf.singlePatchGain_dBi + 10*log10(cfg.totalTxRadiators) - perf.txTaperLoss_dB - perf.feedLoss_dB;
perf.rxGain_count_dBi = perf.singlePatchGain_dBi + 10*log10(cfg.totalRxRadiators) - perf.rxTaperLoss_dB - perf.feedLoss_dB;
perf.txGain_aperture_dBi = 10*log10(4*pi*perf.txActiveArea_m2*perf.apertureEfficiency/(req.lambda0^2)) - perf.feedLoss_dB;
perf.rxGain_aperture_dBi = 10*log10(4*pi*perf.rxActiveArea_m2*perf.apertureEfficiency/(req.lambda0^2)) - perf.feedLoss_dB;
perf.txGainDesign_dBi = min(perf.txGain_count_dBi, perf.txGain_aperture_dBi);
perf.rxGainDesign_dBi = min(perf.rxGain_count_dBi, perf.rxGain_aperture_dBi);
perf.eirp_dBm = req.txTotalPower_dBm + perf.txGainDesign_dBi;
perf.eirp_W   = 1e-3*10^(perf.eirp_dBm/10);
perf.approxAngularRayleigh_deg = (req.lambda0/req.maxAreaXY(1))*180/pi;

fprintf('\n===== Gain and EIRP preliminary estimates =====\n');
fprintf('Single patch assumed gain       = %.1f dBi\n', perf.singlePatchGain_dBi);
fprintf('Tx taper/feed loss              = %.2f / %.2f dB\n', perf.txTaperLoss_dB, perf.feedLoss_dB);
fprintf('Rx taper/feed loss              = %.2f / %.2f dB\n', perf.rxTaperLoss_dB, perf.feedLoss_dB);
fprintf('Tx gain count / aperture / use  = %.2f / %.2f / %.2f dBi\n', perf.txGain_count_dBi, perf.txGain_aperture_dBi, perf.txGainDesign_dBi);
fprintf('Rx gain count / aperture / use  = %.2f / %.2f / %.2f dBi\n', perf.rxGain_count_dBi, perf.rxGain_aperture_dBi, perf.rxGainDesign_dBi);
fprintf('Total Tx input power            = %.2f dBm\n', req.txTotalPower_dBm);
fprintf('Estimated EIRP                  = %.2f dBm = %.1f W isotropic\n', perf.eirp_dBm, perf.eirp_W);
fprintf('Rayleigh angular scale lambda/D = %.2f deg for D=150 mm\n', perf.approxAngularRayleigh_deg);

% Values to copy back into the radar PMCW detectability code.
radarAntennaUpdate = struct();
radarAntennaUpdate.Frequency_Hz = req.f0;
radarAntennaUpdate.Wavelength_m = req.lambda0;
radarAntennaUpdate.PeakPower_W = req.txTotalPower_W;   % CW/PMCW average = peak equivalent in supplied framework
radarAntennaUpdate.TxGain_dBi = perf.txGainDesign_dBi;
radarAntennaUpdate.RxGain_dBi = perf.rxGainDesign_dBi;
radarAntennaUpdate.Gain_dBi = [perf.txGainDesign_dBi perf.rxGainDesign_dBi];
radarAntennaUpdate.TxPowerPerChannel_dBm = req.txPowerPerChannel_dBm;
radarAntennaUpdate.TxChannels = req.numTxChannels;
radarAntennaUpdate.RxChannels = req.numRxChannels;
radarAntennaUpdate.TxRadiators = cfg.totalTxRadiators;
radarAntennaUpdate.RxRadiators = cfg.totalRxRadiators;
radarAntennaUpdate.AzimuthBeamwidth_deg = [beamSummary.TxAzHPBW_deg(1) beamSummary.RxAzHPBW_deg(1)];
radarAntennaUpdate.ElevationBeamwidth_deg = [beamSummary.TxElHPBW_deg(1) beamSummary.RxElHPBW_deg(1)];
radarAntennaUpdate.PanelTilt_deg = cfg.panelTiltDeg;
radarAntennaUpdate.ElementPitch_m = cfg.elementPitch;
radarAntennaUpdate.Patch_W_m = patch.W;
radarAntennaUpdate.Patch_L_m = patch.L;
radarAntennaUpdate.Inset_m = patch.insetY0;
radarAntennaUpdate.FeedLoss_dB = perf.feedLoss_dB;
radarAntennaUpdate.NoiseFigure_dB = req.rxNoiseFigure_dB;
radarAntennaUpdate.SystemTemperature_K = req.receiverSystemTemperature_K;
radarAntennaUpdate.Warning = ['Do not also add array gain elsewhere. In the radar equation use total Tx power at the array input ' ...
    'and the full Tx/Rx aperture gains exactly once.'];

save('radarAntennaUpdate_79GHz.mat','radarAntennaUpdate','req','sub','patch','cfg','ebg','feed','geom','perf','beamSummary');
fprintf('\nSaved radarAntennaUpdate_79GHz.mat\n');

%% 10. Optional Antenna Toolbox single-element geometry sanity check
% This section intentionally tests only one isolated element. Full 1024+1024
% patch EM is normally too large for a quick desktop MoM solve.
try
    if exist('patchMicrostrip','class') == 8 || exist('patchMicrostrip','file') == 2
        antSub = dielectric(sub.name);
        antSub.EpsilonR = sub.er;
        antSub.LossTangent = sub.tand;
        antSub.Thickness = sub.h;
        ant = patchMicrostrip('Length', patch.L, 'Width', patch.W, ...
            'Height', sub.h, 'Substrate', antSub, ...
            'GroundPlaneLength', patch.groundL, 'GroundPlaneWidth', patch.groundW);
        figure('Name','Optional isolated patch geometry','Color','w');
        show(ant); title('Isolated 79 GHz patch, before inset/notch/feed-detail EM refinement');
        figure('Name','Optional isolated patch pattern','Color','w');
        pattern(ant, req.f0); title('Isolated patch pattern estimate');
    else
        fprintf('Antenna Toolbox patchMicrostrip not found; skipped isolated patch EM section.\n');
    end
catch ME
    warning('Optional Antenna Toolbox section failed: %s', ME.message);
end

%% 11. How to modify your supplied PMCW radar code
% In your designRadarPMCW(pmcw) function, use the update values below:
%
% radar.Frequency          = radarAntennaUpdate.Frequency_Hz;
% radar.PeakPower          = radarAntennaUpdate.PeakPower_W;
% radar.Gain               = radarAntennaUpdate.Gain_dBi;
% radar.AzimuthBeamwidth   = radarAntennaUpdate.AzimuthBeamwidth_deg(:);
% radar.ElevationBeamwidth = radarAntennaUpdate.ElevationBeamwidth_deg(:);
% radar.NoiseTemperature   = radarAntennaUpdate.SystemTemperature_K;
% radar.SystemTemperature  = radarAntennaUpdate.SystemTemperature_K;
%
% Do not keep 10 GHz wavelength values if this is a 79 GHz array. Do not use
% 125 W unless your transmitter truly delivers that total array input power.

%% LOCAL FUNCTIONS
function c = physconst_local(name)
    try
        c = physconst(name);
    catch
        switch lower(name)
            case 'lightspeed'
                c = 299792458;
            otherwise
                error('Unknown physical constant: %s', name);
        end
    end
end

function patch = synthesizeRectPatch(f0, er, h)
    c = physconst_local('LightSpeed');
    W = c/(2*f0)*sqrt(2/(er+1));
    eeff = (er+1)/2 + (er-1)/2*(1+12*h/W)^(-0.5);
    deltaL = 0.412*h*((eeff+0.3)*(W/h+0.264))/((eeff-0.258)*(W/h+0.8));
    L = c/(2*f0*sqrt(eeff)) - 2*deltaL;
    lambdaG = c/(f0*sqrt(eeff));

    % Edge resistance is topology-dependent; 250 ohm is a realistic first-pass
    % value for a thin low-er rectangular patch. EM extraction should replace it.
    Redge = 250;
    RinTarget = 50;
    insetY0 = L/pi*acos(sqrt(RinTarget/Redge));

    patch = struct('W',W,'L',L,'eeff',eeff,'deltaL',deltaL,'lambdaG',lambdaG, ...
        'RedgeAssumed',Redge,'RinTarget',RinTarget,'insetY0',insetY0);
end

function w = microstripWidthFromZ0(Z0, h, er)
    % Robust numerical inverse of Hammerstad-Jensen style closed-form Z0.
    lo = 1e-4*h;
    hi = 100*h;
    for it = 1:100
        mid = 0.5*(lo+hi);
        zmid = microstripZ0(mid, h, er);
        if zmid > Z0
            lo = mid; % need wider line to lower impedance
        else
            hi = mid;
        end
    end
    w = 0.5*(lo+hi);
end

function Z0 = microstripZ0(w, h, er)
    u = w/h;
    if u <= 0
        Z0 = inf;
        return;
    end
    if u < 1
        ee = (er+1)/2 + (er-1)/2*(1+12/u)^(-0.5) + 0.04*(1-u)^2;
        Z0 = 60/sqrt(ee)*log(8/u + 0.25*u);
    else
        ee = (er+1)/2 + (er-1)/2*(1+12/u)^(-0.5);
        Z0 = 120*pi/(sqrt(ee)*(u + 1.393 + 0.667*log(u+1.444)));
    end
end

function taper = normalizedTaper2D(nx, ny, sll_dB)
    wx = oneDTaper(nx, sll_dB);
    wy = oneDTaper(ny, sll_dB);
    taper = wx(:)*wy(:).';
    % Normalize so mean power is 1. This preserves total power when applied to
    % elements and makes taper-loss calculation transparent.
    taper = taper/sqrt(mean(abs(taper(:)).^2));
end

function w = oneDTaper(n, sll_dB)
    if n == 1
        w = 1;
        return;
    end
    % Prefer Taylor if Signal Processing Toolbox is present. Fall back to a Hann
    % taper for portability. For n=2 Taylor has little meaning, so use gentle edge.
    try
        if n >= 4 && exist('taylorwin','file') == 2
            w = taylorwin(n, 3, abs(sll_dB)).';
        elseif n >= 4 && exist('chebwin','file') == 2
            w = chebwin(n, abs(sll_dB)).';
        elseif n == 2
            w = [1 1];
        else
            w = hann(n).';
        end
    catch
        if n == 2
            w = [1 1];
        elseif n == 4
            w = [0.55 1 1 0.55];
        else
            w = 0.54 - 0.46*cos(2*pi*(0:n-1)/(n-1));
        end
    end
    w = w/max(w);
end

function T = corporateSplitTable2D(powerMat, Zref, prefix)
    % Returns an ideal split table for a separable/corporate network. Each split
    % assumes both branch inputs are impedance-transformed so their parallel
    % combination equals the parent impedance.
    rows = {};
    rowP = sum(powerMat,2);
    colP = sum(powerMat,1);
    rows = [rows; splitVectorRows(rowP(:).', Zref, [prefix '_row'])]; %#ok<AGROW>
    rows = [rows; splitVectorRows(colP(:).', Zref, [prefix '_col'])]; %#ok<AGROW>
    if isempty(rows)
        T = table();
    else
        T = cell2table(rows, 'VariableNames', {'Node','LeftPowerFrac','RightPowerFrac','LeftBranchZ_ohm','RightBranchZ_ohm'});
    end
end

function rows = splitVectorRows(p, Znode, label)
    p = p(:).';
    p = p/sum(p);
    rows = {};
    if numel(p) <= 1
        return;
    end
    mid = floor(numel(p)/2);
    pL = sum(p(1:mid));
    pR = sum(p(mid+1:end));
    ZL = Znode/pL;
    ZR = Znode/pR;
    rows(end+1,:) = {label, pL, pR, ZL, ZR}; %#ok<AGROW>
    rows = [rows; splitVectorRows(p(1:mid)/pL, ZL, [label 'L'])]; %#ok<AGROW>
    rows = [rows; splitVectorRows(p(mid+1:end)/pR, ZR, [label 'R'])]; %#ok<AGROW>
end

function warnHighZ(T, maxZ, minZ)
    if isempty(T) || height(T)==0
        return;
    end
    vals = [T.LeftBranchZ_ohm; T.RightBranchZ_ohm];
    if any(vals > maxZ) || any(vals < minZ)
        warning(['Some ideal branch impedances are outside %.0f-%.0f ohm. ' ...
            'At 79 GHz, implement these as unequal Wilkinson dividers plus bounded-impedance transformers, not raw star branches.'], minZ, maxZ);
    end
end

function geom = buildCantedArrayGeometry(req, cfg, patch)
    nPanelsX = cfg.panelCountXY(1);
    nPanelsY = cfg.panelCountXY(2);
    panelCentersXY = zeros(cfg.numPanels,2);
    idx = 1;
    for ix = 1:nPanelsX
        for iy = 1:nPanelsY
            x = (ix - (nPanelsX+1)/2)*cfg.panelSide;
            y = (iy - (nPanelsY+1)/2)*cfg.panelSide;
            panelCentersXY(idx,:) = [x y];
            idx = idx + 1;
        end
    end

    txPos = []; rxPos = [];
    txNorm = []; rxNorm = [];
    txW = []; rxW = [];
    txChan = []; rxChan = [];
    panelInfo = struct([]);

    pTilt = deg2rad(cfg.panelTiltDeg);
    globalZ = [0 0 1];

    txChannelBase = 0;
    rxChannelBase = 0;
    for p = 1:cfg.numPanels
        cxy = panelCentersXY(p,:);
        center = [cxy 0];
        radial = [cxy 0];
        if norm(radial) < eps
            radial = [1 0 0];
        else
            radial = radial/norm(radial);
        end
        tangent = [-radial(2) radial(1) 0];
        tangent = tangent/norm(tangent);
        normal = radial*sin(pTilt) + globalZ*cos(pTilt);
        normal = normal/norm(normal);
        vAxis = cross(normal, tangent);
        vAxis = vAxis/norm(vAxis);
        uAxis = tangent;

        panelInfo(p).center = center; %#ok<AGROW>
        panelInfo(p).normal = normal;
        panelInfo(p).u = uAxis;
        panelInfo(p).v = vAxis;

        [txBlockPos, txBlockNorm, txBlockW, txBlockChan] = buildBlock(center, uAxis, vAxis, normal, ...
            [0 -cfg.txRxBlockOffset], cfg, p, txChannelBase, patch, 'tx');
        [rxBlockPos, rxBlockNorm, rxBlockW, rxBlockChan] = buildBlock(center, uAxis, vAxis, normal, ...
            [0 +cfg.txRxBlockOffset], cfg, p, rxChannelBase, patch, 'rx');

        txPos = [txPos; txBlockPos]; %#ok<AGROW>
        rxPos = [rxPos; rxBlockPos]; %#ok<AGROW>
        txNorm = [txNorm; txBlockNorm]; %#ok<AGROW>
        rxNorm = [rxNorm; rxBlockNorm]; %#ok<AGROW>
        txW = [txW; txBlockW]; %#ok<AGROW>
        rxW = [rxW; rxBlockW]; %#ok<AGROW>
        txChan = [txChan; txBlockChan]; %#ok<AGROW>
        rxChan = [rxChan; rxBlockChan]; %#ok<AGROW>

        txChannelBase = txChannelBase + cfg.txChannelsPerPanel;
        rxChannelBase = rxChannelBase + cfg.rxChannelsPerPanel;
    end

    geom.tx.pos = txPos;
    geom.rx.pos = rxPos;
    geom.tx.normal = txNorm;
    geom.rx.normal = rxNorm;
    geom.tx.weights = txW/sqrt(mean(abs(txW).^2));
    geom.rx.weights = rxW/sqrt(mean(abs(rxW).^2));
    geom.tx.channel = txChan;
    geom.rx.channel = rxChan;
    geom.panel = panelInfo;
end

function [pos, normals, weights, channels] = buildBlock(center, uAxis, vAxis, normal, localCenterUV, cfg, panelIdx, channelBase, patch, blockName)
    %#ok<INUSD>
    nx = cfg.radiatorsPerPanelXY(1);
    ny = cfg.radiatorsPerPanelXY(2);
    d = cfg.elementPitch;
    xOffsets = ((0:nx-1) - (nx-1)/2)*d;
    yOffsets = ((0:ny-1) - (ny-1)/2)*d;

    pos = zeros(nx*ny,3);
    normals = repmat(normal, nx*ny, 1);
    weights = zeros(nx*ny,1);
    channels = zeros(nx*ny,1);
    k = 1;
    for ix = 1:nx
        for iy = 1:ny
            localUV = localCenterUV + [xOffsets(ix) yOffsets(iy)];
            pos(k,:) = center + localUV(1)*uAxis + localUV(2)*vAxis + 0.5e-3*normal;

            tileIx = floor((ix-1)/cfg.tileSize(1)) + 1;
            tileIy = floor((iy-1)/cfg.tileSize(2)) + 1;
            elemIx = mod(ix-1, cfg.tileSize(1)) + 1;
            elemIy = mod(iy-1, cfg.tileSize(2)) + 1;
            chLocal = (tileIy-1)*cfg.channelGridPerPanel(1) + tileIx;
            channels(k) = channelBase + chLocal;

            panelX = 1 + mod(panelIdx-1,2);
            panelY = 1 + floor((panelIdx-1)/2);
            pW = cfg.panelTaper(panelX,panelY);
            cW = cfg.channelTaper(tileIx,tileIy);
            eW = cfg.tileTaper(elemIx,elemIy);
            weights(k) = pW*cW*eW;
            k = k + 1;
        end
    end
end

function plotCantedArray3D(geom, cfg, patchGeom, ebg)
    hold on; axis equal; grid on; view(42,28);
    xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');

    % Draw panel boards.
    for p = 1:numel(geom.panel)
        C = geom.panel(p).center;
        u = geom.panel(p).u;
        v = geom.panel(p).v;
        s = cfg.panelSide/2;
        corners = [C - s*u - s*v; C + s*u - s*v; C + s*u + s*v; C - s*u + s*v]*1e3;
        patchSurf = patch('Faces',[1 2 3 4],'Vertices',corners, ...
            'FaceAlpha',0.10,'EdgeAlpha',0.35,'FaceColor',[0.8 0.8 0.8]); %#ok<NASGU>
    end

    % Draw radiating patches as small rectangles. Use simplified rectangles on
    % the local planes. Blue = Tx, red = Rx. This can be heavy; decimate if needed.
    drawPatchSet(geom.tx.pos, geom.tx.normal, patchGeom.W, patchGeom.L, [0.1 0.35 0.95]);
    drawPatchSet(geom.rx.pos, geom.rx.normal, patchGeom.W, patchGeom.L, [0.95 0.25 0.15]);

    % Channel centers.
    txCenters = channelCenters(geom.tx.pos, geom.tx.channel);
    rxCenters = channelCenters(geom.rx.pos, geom.rx.channel);
    scatter3(txCenters(:,1)*1e3, txCenters(:,2)*1e3, txCenters(:,3)*1e3, 24, 'filled', 'MarkerFaceColor',[0 0 0.8], 'DisplayName','Tx RF-channel phase centers');
    scatter3(rxCenters(:,1)*1e3, rxCenters(:,2)*1e3, rxCenters(:,3)*1e3, 24, 'filled', 'MarkerFaceColor',[0.8 0 0], 'DisplayName','Rx RF-channel phase centers');

    % EBG indication as perimeter dots around each panel.
    for p = 1:numel(geom.panel)
        C = geom.panel(p).center; u = geom.panel(p).u; v = geom.panel(p).v; n = geom.panel(p).normal;
        s = cfg.panelSide/2 - 2e-3;
        pts = [];
        vals = -s:ebg.pitch:s;
        for a = vals
            pts = [pts; C + a*u - s*v + 0.7e-3*n; C + a*u + s*v + 0.7e-3*n]; %#ok<AGROW>
            pts = [pts; C - s*u + a*v + 0.7e-3*n; C + s*u + a*v + 0.7e-3*n]; %#ok<AGROW>
        end
        scatter3(pts(:,1)*1e3, pts(:,2)*1e3, pts(:,3)*1e3, 4, [0.15 0.15 0.15], 'filled', 'HandleVisibility','off');
    end
    legend('Location','bestoutside');
end

function drawPatchSet(pos, normals, W, L, color)
    N = size(pos,1);
    for i = 1:N
        n = normals(i,:); n = n/norm(n);
        tmp = [1 0 0];
        if abs(dot(tmp,n)) > 0.85
            tmp = [0 1 0];
        end
        u = cross(n,tmp); u = u/norm(u);
        v = cross(n,u); v = v/norm(v);
        C = pos(i,:);
        corners = [C - W/2*u - L/2*v; C + W/2*u - L/2*v; C + W/2*u + L/2*v; C - W/2*u + L/2*v]*1e3;
        patch('Faces',[1 2 3 4],'Vertices',corners,'FaceColor',color,'EdgeColor','none','FaceAlpha',0.75,'HandleVisibility','off');
    end
end

function centers = channelCenters(pos, ch)
    ids = unique(ch);
    centers = zeros(numel(ids),3);
    for k = 1:numel(ids)
        centers(k,:) = mean(pos(ch==ids(k),:),1);
    end
end

function [patdB, thetaDeg] = arrayFactorCut(pos, normals, weights, f0, thetaDeg, cut, scanXDeg, scanYDeg, elemQ)
    c = physconst_local('LightSpeed');
    k0 = 2*pi*f0/c;
    s0 = directionFromBoresight(scanXDeg, scanYDeg);
    steer = exp(-1j*k0*(pos*s0(:)));
    w = weights(:).*steer;
    AF = zeros(size(thetaDeg));
    for it = 1:numel(thetaDeg)
        th = thetaDeg(it);
        switch lower(cut)
            case 'az'
                s = directionFromBoresight(th, 0);
            case 'el'
                s = directionFromBoresight(0, th);
            otherwise
                error('cut must be az or el');
        end
        elem = max(normals*s(:), 0).^elemQ;
        AF(it) = sum(w .* elem .* exp(1j*k0*(pos*s(:))));
    end
    P = abs(AF).^2;
    P = P/max(P + eps);
    patdB = 10*log10(P + 1e-15);
end

function s = directionFromBoresight(thetaXDeg, thetaYDeg)
    % Direction vector for small-angle scan from +z boresight.
    ux = sind(thetaXDeg);
    uy = sind(thetaYDeg);
    uz2 = 1 - ux^2 - uy^2;
    uz = sqrt(max(uz2, 0));
    s = [ux uy uz];
    s = s/norm(s);
end

function bw = hpbw(theta, patdB)
    [~, imax] = max(patdB);
    left = find(patdB(1:imax) <= -3, 1, 'last');
    rightRel = find(patdB(imax:end) <= -3, 1, 'first');
    if isempty(left) || isempty(rightRel)
        bw = NaN;
        return;
    end
    right = imax + rightRel - 1;
    bw = theta(right) - theta(left);
end

function sll = estimateSLL(theta, patdB)
    bw = hpbw(theta, patdB);
    if isnan(bw)
        mask = abs(theta) > 5;
    else
        [~, imax] = max(patdB);
        th0 = theta(imax);
        mask = abs(theta - th0) > max(1.2*bw, 2.0);
    end
    if ~any(mask)
        sll = NaN;
    else
        sll = max(patdB(mask));
    end
end

function L = taperLossdB(w)
    % Directivity loss of taper relative to uniform excitation for fixed total
    % radiated power: L = 10log10(N*sum(|w|^2)/|sum(w)|^2).
    w = w(:);
    N = numel(w);
    L = 10*log10(N*sum(abs(w).^2)/(abs(sum(w))^2 + eps));
end
