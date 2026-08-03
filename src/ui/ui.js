import { PatientMonitor } from './patientMonitor.js';
import { initCArmPreview, renderCArmPreview, cArmPreviewGroup, cArmPreviewGantry, cArmPreviewDetectorAssembly, cArmPreviewTable } from './carmPreview.js';
import { setupCArmControls } from '../carmControls.js';
import { AutomaticWithdrawalController } from './automaticWithdrawalController.js';
import { setBendingStiffness, setWallFriction, setSmoothingIterations } from '../physics/elasticRod.js';
import {
  GUIDEWIRE_DIAMETER_IN,
  GUIDEWIRE_DIAMETER_MM,
  INTRODUCER_SHEATH_DIAMETER_MM,
  INTRODUCER_SHEATH_FRENCH,
  PIGTAIL_CATHETER_DIAMETER_MM,
  PIGTAIL_CATHETER_FRENCH
} from '../toolDimensions.js';

// Initializes all UI elements and event listeners.
// Expects options with references and callbacks to interact with the simulator.
// Returns helpers to query UI-driven state and update UI readouts.
export function initUI(options) {
  const {
    camera,
    cameraRadius,
    vessel,
    voxelGroup,
    displayMaterial,
    blendMaterial,
    wireMaterial,
    onStartInjection,
    onStopInjection,
    onModeChange,
    onDebugLayerChange,
    onContrastHemodynamicsChange,
    onContrastInjectionParametersChange,
    onInjectionRequestChange,
    onPrepareCatheterAorta,
    onReproduceIliacContrastBug,
    onReproduceRetrogradeGap,
    onReproduceArchBolus,
    onStartBrowserBenchmark,
    onStopBrowserBenchmark,
    onRequestDsaMask,
    onToggleDsa,
    onCaptureRoadmap,
    onToggleRoadmap,
    onClearRoadmap,
    onStartDsaRecording,
    onStopDsaRecording,
    onSelectDsaSequence,
    onSelectDsaFrame,
    onUseBestDsaFrame,
    onToggleDsaCine,
    onStopDsaCine,
  } = options;

  // Patient monitor
  const monitor = new PatientMonitor(
    document.getElementById('ecgCanvas'),
    document.getElementById('bpCanvas'),
    document.getElementById('hrValue'),
    document.getElementById('bpValue'),
    {
      spo2Elem: document.getElementById('spo2Value'),
      mapElem: document.getElementById('mapValue'),
      rrElem: document.getElementById('rrValue'),
      rhythmElem: document.getElementById('monitorRhythm'),
      clockElem: document.getElementById('monitorClock'),
    }
  );

  // C-arm UI preview + controls
  const cArmPreview = initCArmPreview();
  const cArmControls = setupCArmControls(
    camera,
    vessel,
    cameraRadius,
    cArmPreview?.group || cArmPreviewGroup,
    cArmPreview?.gantry || cArmPreviewGantry,
    cArmPreview?.detectorAssembly || cArmPreviewDetectorAssembly,
    cArmPreview?.lift,
    cArmPreview?.table || cArmPreviewTable,
    renderCArmPreview
  );

  // UI elements
  const bendSlider = document.getElementById('stiffness');
  const staticFricSlider = document.getElementById('staticFriction');
  const kineticFricSlider = document.getElementById('kineticFriction');
  const smoothIterSlider = document.getElementById('smoothIterations');
  const modeToggle = document.getElementById('modeToggle');
  const voxelRenderToggle = document.getElementById('renderVoxels');
  const debugStlModelToggle = document.getElementById('showDebugStlModel');
  const debugLumenCastToggle = document.getElementById('showDebugLumenCast');
  const debugSectionsToggle = document.getElementById('showDebugSections');
  const debugCenterlineToggle = document.getElementById('showDebugCenterline');
  const debugCapsulesToggle = document.getElementById('showDebugCapsules');
  const cardiacOutputSlider = document.getElementById('cardiacOutput');
  const contrastHeartRateSlider = document.getElementById('contrastHeartRate');
  const injectButton = document.getElementById('injectContrast');
  const stopInjectButton = document.getElementById('stopInjection');
  const injRateSlider = document.getElementById('injRate');
  const injDurationOutput = document.getElementById('injDuration');
  const injVolumeSlider = document.getElementById('injVolume');
  const injectionVolumePresetButtons = Array.from(
    document.querySelectorAll('[data-injection-volume]')
  );
  const injSourceSelect = document.getElementById('injSource');
  const injectionSourceStatusEl = document.getElementById('injectionSourceStatus');
  const injectionHydraulicSummaryEl = document.getElementById('injectionHydraulicSummary');
  const injActualRateOutput = document.getElementById('injActualRate');
  const injPressureOutput = document.getElementById('injPressure');
  const injPressureWarningOutput = document.getElementById('injPressureWarning');
  const injectorPressureLimitSlider = document.getElementById('injectorPressureLimit');
  const contrastViscositySlider = document.getElementById('contrastViscosity');
  const sheathHydraulicLengthSlider = document.getElementById('sheathHydraulicLength');
  const sheathInnerDiameterSlider = document.getElementById('sheathInnerDiameter');
  const sheathPressureRatingSlider = document.getElementById('sheathPressureRating');
  const berensteinHydraulicLengthSlider = document.getElementById('berensteinHydraulicLength');
  const berensteinInnerDiameterSlider = document.getElementById('berensteinInnerDiameter');
  const berensteinPressureRatingSlider = document.getElementById('berensteinPressureRating');
  const pigtailHydraulicLengthSlider = document.getElementById('pigtailHydraulicLength');
  const pigtailInnerDiameterSlider = document.getElementById('pigtailInnerDiameter');
  const pigtailPressureRatingSlider = document.getElementById('pigtailPressureRating');
  const autoExposureToggle = document.getElementById('autoExposureToggle');
  const persistenceSlider = document.getElementById('persistence');
  const pulseRateSlider = document.getElementById('pulseRate');
  const noiseSlider = document.getElementById('noiseLevel');
  const scatterStrengthSlider = document.getElementById('scatterStrength');
  const collimationSlider = document.getElementById('collimation');
  const imageBrightnessSlider = document.getElementById('imageBrightness');
  const imageContrastSlider = document.getElementById('imageContrast');
  const edgeEnhancementSlider = document.getElementById('edgeEnhancement');
  const boneVisibilitySlider = document.getElementById('boneVisibility');
  const contrastOpacitySlider = document.getElementById('opacityScale');
  const contrastGainSlider = document.getElementById('gain');
  const acquireDsaMaskButton = document.getElementById('acquireDsaMask');
  const toggleDsaButton = document.getElementById('toggleDsa');
  const recordDsaSequenceButton = document.getElementById('recordDsaSequence');
  const captureRoadmapButton = document.getElementById('captureRoadmap');
  const toggleRoadmapButton = document.getElementById('toggleRoadmap');
  const clearRoadmapButton = document.getElementById('clearRoadmap');
  const dsaSequenceSelect = document.getElementById('dsaSequenceSelect');
  const dsaFrameSelect = document.getElementById('dsaFrameSelect');
  const useBestDsaFrameButton = document.getElementById('useBestDsaFrame');
  const dsaFrameInfoEl = document.getElementById('dsaFrameInfo');
  const dsaSequenceGalleryEl = document.getElementById('dsaSequenceGallery');
  const dsaCineControlsEl = document.getElementById('dsaCineControls');
  const dsaCineStatusEl = document.getElementById('dsaCineStatus');
  const dsaCinePlayPauseButton = document.getElementById('dsaCinePlayPause');
  const dsaCineStopButton = document.getElementById('dsaCineStop');
  const roadmapOpacitySlider = document.getElementById('roadmapOpacity');
  const roadmapBackgroundSlider = document.getElementById('roadmapBackground');
  const dsaGainSlider = document.getElementById('dsaGain');
  const dsaRoadmapStatusEl = document.getElementById('dsaRoadmapStatus');
  const imagingModeBadgeEl = document.getElementById('imagingModeBadge');
  const insertedLengthEl = document.getElementById('insertedLength');
  const catheterLengthEl = document.getElementById('catheterLength');
  const guidewireAutoWithdrawButton = document.getElementById('guidewireAutoWithdraw');
  const catheterAutoWithdrawButton = document.getElementById('catheterAutoWithdraw');
  const catheterAdvanceButton = document.getElementById('catheterAdvance');
  const catheterWithdrawButton = document.getElementById('catheterWithdraw');
  const catheterRotateLeftButton = document.getElementById('catheterRotateLeft');
  const catheterRotateRightButton = document.getElementById('catheterRotateRight');
  const catheterTypeSelect = document.getElementById('catheterType');
  const guidewireTypeSelect = document.getElementById('guidewireType');
  const catheterTypeStatusEl = document.getElementById('catheterTypeStatus');
  const guidewireTypeStatusEl = document.getElementById('guidewireTypeStatus');
  const doseDisplayEl = document.getElementById('currentDose');
  const currentKVEl = document.getElementById('currentKV');
  const currentMAEl = document.getElementById('currentMA');
  const guidewireResistanceEl = document.getElementById('guidewireResistanceStatus');
  const guidewireResistanceReasonEl = document.getElementById('guidewireResistanceReason');
  const guidewireResistanceValueEl = document.getElementById('guidewireResistanceValue');
  const guidewireResistanceFillEl = document.getElementById('guidewireResistanceFill');
  const guidewireDiagnosticsEl = document.getElementById('guidewireDiagnostics');
  const contrastDiagnosticsEl = document.getElementById('contrastDiagnostics');
  const guidewireDiameterEl = document.getElementById('guidewireDiameter');
  const sheathDiameterEl = document.getElementById('sheathDiameter');
  const catheterDiameterEl = document.getElementById('catheterDiameter');
  const perfStatsEl = document.getElementById('perfStats');
  const prepareCatheterAortaButton = document.getElementById('prepareCatheterAorta');
  const reproduceIliacContrastBugButton = document.getElementById('reproduceIliacContrastBug');
  const reproduceRetrogradeGapButton = document.getElementById('reproduceRetrogradeGap');
  const reproduceArchBolusButton = document.getElementById('reproduceArchBolus');
  const catheterAortaSetupStatusEl = document.getElementById('catheterAortaSetupStatus');
  const runBrowserBenchmarkSmokeButton = document.getElementById('runBrowserBenchmarkSmoke');
  const runBrowserBenchmarkFullButton = document.getElementById('runBrowserBenchmarkFull');
  const stopBrowserBenchmarkButton = document.getElementById('stopBrowserBenchmark');
  const browserBenchmarkStatusEl = document.getElementById('browserBenchmarkStatus');
  const browserBenchmarkReportEl = document.getElementById('browserBenchmarkReport');

  // Initial UI state
  if (guidewireDiameterEl) {
    guidewireDiameterEl.textContent = `${GUIDEWIRE_DIAMETER_IN.toFixed(3)}" · ${GUIDEWIRE_DIAMETER_MM.toFixed(3)} mm`;
  }
  if (sheathDiameterEl) {
    sheathDiameterEl.textContent = `${INTRODUCER_SHEATH_FRENCH}F · ${INTRODUCER_SHEATH_DIAMETER_MM.toFixed(3)} mm`;
  }
  if (catheterDiameterEl) {
    catheterDiameterEl.textContent = `${PIGTAIL_CATHETER_FRENCH}F · ${PIGTAIL_CATHETER_DIAMETER_MM.toFixed(3)} mm`;
  }

  if (voxelRenderToggle) {
    voxelGroup.visible = voxelRenderToggle.checked;
  }
  const debugLayerState = {
    stlModel: debugStlModelToggle?.checked ?? true,
    lumenCast: debugLumenCastToggle?.checked ?? false,
    sections: debugSectionsToggle?.checked ?? false,
    centerline: debugCenterlineToggle?.checked ?? true,
    capsules: debugCapsulesToggle?.checked ?? false
  };

  function emitDebugLayerChange() {
    if (typeof onDebugLayerChange === 'function') {
      onDebugLayerChange({ ...debugLayerState });
    }
  }

  debugStlModelToggle?.addEventListener('change', e => {
    debugLayerState.stlModel = e.target.checked;
    emitDebugLayerChange();
  });
  debugLumenCastToggle?.addEventListener('change', e => {
    debugLayerState.lumenCast = e.target.checked;
    emitDebugLayerChange();
  });
  debugSectionsToggle?.addEventListener('change', e => {
    debugLayerState.sections = e.target.checked;
    emitDebugLayerChange();
  });
  debugCenterlineToggle?.addEventListener('change', e => {
    debugLayerState.centerline = e.target.checked;
    emitDebugLayerChange();
  });
  debugCapsulesToggle?.addEventListener('change', e => {
    debugLayerState.capsules = e.target.checked;
    emitDebugLayerChange();
  });
  emitDebugLayerChange();

  let insertedLengthCm = 0;
  let catheterLengthCm = 0;
  let insertedLengthTenths = -1;
  let catheterLengthTenths = -1;
  let doseTenths = -1;
  let roundedKv = -1;
  let maTenths = -1;
  let insertedLengthDisplay = '';
  let catheterLengthDisplay = '';
  let doseDisplay = '';
  let kvDisplay = '';
  let maDisplay = '';
  let resistanceModerate = null;
  let resistanceStrong = null;
  let resistancePercent = -1;
  let resistanceReason = '';
  let selectedCatheterType = catheterTypeSelect?.value || 'pigtail';
  let selectedGuidewireType = guidewireTypeSelect?.value || 'glidewire';
  const TOOL_SELECTION_UNLOCK_EPSILON_CM = 0.05;
  const guidewireAutoWithdraw = new AutomaticWithdrawalController({
    emptyThresholdCm: TOOL_SELECTION_UNLOCK_EPSILON_CM
  });
  const catheterAutoWithdraw = new AutomaticWithdrawalController({
    emptyThresholdCm: TOOL_SELECTION_UNLOCK_EPSILON_CM
  });

  function updateAutoWithdrawButton(button, controller) {
    if (!button) return;
    button.disabled = controller.disabled;
    button.classList.toggle('active', controller.active);
    button.setAttribute('aria-pressed', String(controller.active));
    button.textContent = controller.active ? 'Zatrzymaj' : 'Wysuń';
  }

  function stopGuidewireAutoWithdraw() {
    guidewireAutoWithdraw.cancel();
    updateAutoWithdrawButton(guidewireAutoWithdrawButton, guidewireAutoWithdraw);
  }

  function stopCatheterAutoWithdraw() {
    catheterAutoWithdraw.cancel();
    updateAutoWithdrawButton(catheterAutoWithdrawButton, catheterAutoWithdraw);
  }

  function updateSelectLock(select, statusEl, locked, insertedCm) {
    if (select) {
      if (select.disabled !== locked) select.disabled = locked;
      const title = locked ? 'Withdraw to 0 cm before changing selection' : '';
      if (select.title !== title) select.title = title;
    }
    if (statusEl) {
      const text = locked ? `${insertedCm.toFixed(1)} cm inserted` : 'Ready';
      if (statusEl.textContent !== text) statusEl.textContent = text;
      if (statusEl.classList.contains('locked') !== locked) {
        statusEl.classList.toggle('locked', locked);
      }
    }
  }

  function updateToolSelectionLocks() {
    updateSelectLock(
      guidewireTypeSelect,
      guidewireTypeStatusEl,
      insertedLengthCm > TOOL_SELECTION_UNLOCK_EPSILON_CM,
      insertedLengthCm
    );
    updateSelectLock(
      catheterTypeSelect,
      catheterTypeStatusEl,
      catheterLengthCm > TOOL_SELECTION_UNLOCK_EPSILON_CM,
      catheterLengthCm
    );
    updateAutoWithdrawButton(
      guidewireAutoWithdrawButton,
      guidewireAutoWithdraw
    );
    updateAutoWithdrawButton(
      catheterAutoWithdrawButton,
      catheterAutoWithdraw
    );
  }

  catheterTypeSelect?.addEventListener('change', e => {
    selectedCatheterType = e.target.value;
  });
  guidewireTypeSelect?.addEventListener('change', e => {
    selectedGuidewireType = e.target.value;
  });
  guidewireAutoWithdrawButton?.addEventListener('click', () => {
    guidewireAutoWithdraw.toggle();
    updateAutoWithdrawButton(guidewireAutoWithdrawButton, guidewireAutoWithdraw);
  });
  catheterAutoWithdrawButton?.addEventListener('click', () => {
    catheterAutoWithdraw.toggle();
    updateAutoWithdrawButton(catheterAutoWithdrawButton, catheterAutoWithdraw);
  });
  updateToolSelectionLocks();

  prepareCatheterAortaButton?.addEventListener('click', () => {
    if (typeof onPrepareCatheterAorta === 'function') onPrepareCatheterAorta();
  });
  reproduceIliacContrastBugButton?.addEventListener('click', () => {
    if (typeof onReproduceIliacContrastBug === 'function') onReproduceIliacContrastBug();
  });
  reproduceRetrogradeGapButton?.addEventListener('click', () => {
    if (typeof onReproduceRetrogradeGap === 'function') onReproduceRetrogradeGap();
  });
  reproduceArchBolusButton?.addEventListener('click', () => {
    if (typeof onReproduceArchBolus === 'function') onReproduceArchBolus();
  });
  runBrowserBenchmarkSmokeButton?.addEventListener('click', () => {
    if (typeof onStartBrowserBenchmark === 'function') onStartBrowserBenchmark(5000);
  });
  runBrowserBenchmarkFullButton?.addEventListener('click', () => {
    if (typeof onStartBrowserBenchmark === 'function') onStartBrowserBenchmark(600000);
  });
  stopBrowserBenchmarkButton?.addEventListener('click', () => {
    if (typeof onStopBrowserBenchmark === 'function') onStopBrowserBenchmark();
  });

  const controlTabs = Array.from(document.querySelectorAll('[data-control-tab]'));
  const controlPanels = Array.from(document.querySelectorAll('[data-control-panel]'));
  if (controlTabs.length && controlPanels.length) {
    const activateControlTab = tabName => {
      controlTabs.forEach(tab => {
        const active = tab.dataset.controlTab === tabName;
        tab.classList.toggle('active', active);
        tab.setAttribute('aria-selected', active ? 'true' : 'false');
      });
      controlPanels.forEach(panel => {
        panel.classList.toggle('active', panel.dataset.controlPanel === tabName);
      });
    };
    controlTabs.forEach(tab => {
      tab.addEventListener('click', () => activateControlTab(tab.dataset.controlTab));
    });
  }

  // Avoid sticky focus on sliders
  const sliders = [
    bendSlider,
    staticFricSlider,
    kineticFricSlider,
    smoothIterSlider,
    persistenceSlider,
    pulseRateSlider,
    noiseSlider,
    scatterStrengthSlider,
    collimationSlider,
    imageBrightnessSlider,
    imageContrastSlider,
    edgeEnhancementSlider,
    boneVisibilitySlider,
    contrastOpacitySlider,
    contrastGainSlider,
    injVolumeSlider,
    injRateSlider,
    cardiacOutputSlider,
    contrastHeartRateSlider
  ].filter(Boolean);
  sliders.forEach(s => s.addEventListener('change', () => s.blur()));

  if (voxelRenderToggle) {
    voxelRenderToggle.addEventListener('change', e => {
      voxelGroup.visible = e.target.checked;
    });
  }

  // Display current values next to each slider
  document.querySelectorAll('#controls input[type="range"], #carm-controls input[type="range"]').forEach(slider => {
    const valueLabel = slider.nextElementSibling;
    if (!valueLabel) return;
    const update = () => { valueLabel.textContent = slider.value; };
    update();
    slider.addEventListener('input', update);
  });

  const emitContrastHemodynamics = () => {
    const cardiacOutputMlPerMin = parseFloat(cardiacOutputSlider?.value || '5000');
    const heartRateBpm = parseFloat(contrastHeartRateSlider?.value || '72');
    const cardiacOutputValue = cardiacOutputSlider?.nextElementSibling;
    const heartRateValue = contrastHeartRateSlider?.nextElementSibling;
    if (cardiacOutputValue) {
      cardiacOutputValue.textContent = `${(cardiacOutputMlPerMin / 1000).toFixed(1)} l/min`;
    }
    if (heartRateValue) heartRateValue.textContent = `${Math.round(heartRateBpm)}/min`;
    onContrastHemodynamicsChange?.({ cardiacOutputMlPerMin, heartRateBpm });
  };
  cardiacOutputSlider?.addEventListener('input', emitContrastHemodynamics);
  contrastHeartRateSlider?.addEventListener('input', emitContrastHemodynamics);
  emitContrastHemodynamics();

  const hydraulicParameterControls = [
    [injectorPressureLimitSlider, value => `${value.toFixed(0)} psi`],
    [contrastViscositySlider, value => `${value.toFixed(1)} mPa·s`],
    [sheathHydraulicLengthSlider, value => `${value.toFixed(0)} mm`],
    [sheathInnerDiameterSlider, value => `${value.toFixed(2)} mm`],
    [sheathPressureRatingSlider, value => `${value.toFixed(0)} psi`],
    [berensteinHydraulicLengthSlider, value => `${value.toFixed(0)} mm`],
    [berensteinInnerDiameterSlider, value => `${value.toFixed(2)} mm`],
    [berensteinPressureRatingSlider, value => `${value.toFixed(0)} psi`],
    [pigtailHydraulicLengthSlider, value => `${value.toFixed(0)} mm`],
    [pigtailInnerDiameterSlider, value => `${value.toFixed(2)} mm`],
    [pigtailPressureRatingSlider, value => `${value.toFixed(0)} psi`]
  ];
  const emitContrastInjectionParameters = () => {
    for (const [control, format] of hydraulicParameterControls) {
      const value = parseFloat(control?.value || '0');
      const valueEl = control?.nextElementSibling;
      if (valueEl) valueEl.textContent = format(value);
    }
    onContrastInjectionParametersChange?.({
      maximumPressurePsi: parseFloat(
        injectorPressureLimitSlider?.value || '1200'
      ),
      viscosityPaS: parseFloat(
        contrastViscositySlider?.value || '6.3'
      ) / 1000,
      deviceProfiles: {
        sheath: {
          lengthMm: parseFloat(
            sheathHydraulicLengthSlider?.value || '110'
          ),
          innerDiameterMm: parseFloat(
            sheathInnerDiameterSlider?.value || '1.8'
          ),
          maximumPressurePsi: parseFloat(
            sheathPressureRatingSlider?.value || '300'
          )
        },
        berenstein: {
          lengthMm: parseFloat(
            berensteinHydraulicLengthSlider?.value || '1000'
          ),
          innerDiameterMm: parseFloat(
            berensteinInnerDiameterSlider?.value || '0.97'
          ),
          maximumPressurePsi: parseFloat(
            berensteinPressureRatingSlider?.value || '1050'
          )
        },
        pigtail: {
          lengthMm: parseFloat(
            pigtailHydraulicLengthSlider?.value || '1000'
          ),
          innerDiameterMm: parseFloat(
            pigtailInnerDiameterSlider?.value || '0.97'
          ),
          maximumPressurePsi: parseFloat(
            pigtailPressureRatingSlider?.value || '1200'
          )
        }
      }
    });
    onInjectionRequestChange?.({
      source: injSourceSelect?.value || 'sheath',
      rateMlPerSec: parseFloat(injRateSlider?.value || '0'),
      volumeMl: parseFloat(injVolumeSlider?.value || '0')
    });
  };
  for (const [control] of hydraulicParameterControls) {
    control?.addEventListener('input', emitContrastInjectionParameters);
  }
  emitContrastInjectionParameters();

  let lastInjectionHydraulicPreview = null;
  const emitInjectionRequest = () => {
    onInjectionRequestChange?.({
      source: injSourceSelect?.value || 'sheath',
      rateMlPerSec: parseFloat(injRateSlider?.value || '0'),
      volumeMl: parseFloat(injVolumeSlider?.value || '0')
    });
  };
  const updateInjectionDuration = () => {
    if (!injDurationOutput) return;
    const volume = parseFloat(injVolumeSlider?.value || '0');
    const requestedRate = parseFloat(injRateSlider?.value || '0');
    const source = injSourceSelect?.value || 'sheath';
    const previewMatches =
      lastInjectionHydraulicPreview?.valid &&
      lastInjectionHydraulicPreview.source === source &&
      Math.abs(
        lastInjectionHydraulicPreview.requestedRateMlPerSec - requestedRate
      ) < 1e-6;
    const rate = previewMatches
      ? lastInjectionHydraulicPreview.actualRateMlPerSec
      : requestedRate;
    const duration = rate > 0 ? volume / rate : 0;
    injDurationOutput.value = `${duration.toFixed(2)} s`;
    injDurationOutput.textContent = injDurationOutput.value;
    emitInjectionRequest();
  };
  const updateInjectionVolumePresetState = () => {
    const selectedVolume = parseFloat(injVolumeSlider?.value || '0');
    for (const button of injectionVolumePresetButtons) {
      const presetVolume = parseFloat(
        button.dataset.injectionVolume || '0'
      );
      const active = Math.abs(selectedVolume - presetVolume) < 1e-6;
      button.classList.toggle('active', active);
      button.setAttribute('aria-pressed', String(active));
    }
  };
  injVolumeSlider?.addEventListener('input', () => {
    updateInjectionVolumePresetState();
    updateInjectionDuration();
  });
  for (const button of injectionVolumePresetButtons) {
    button.addEventListener('click', () => {
      if (!injVolumeSlider) return;
      injVolumeSlider.value = button.dataset.injectionVolume || '0';
      injVolumeSlider.dispatchEvent(new Event('input', {
        bubbles: true
      }));
    });
  }
  injRateSlider?.addEventListener('input', updateInjectionDuration);
  injSourceSelect?.addEventListener('change', updateInjectionDuration);
  updateInjectionVolumePresetState();
  updateInjectionDuration();
  if (injectButton) injectButton.disabled = true;

  // Toggle visibility of control sections
  document.querySelectorAll('.section-header').forEach(header => {
    header.addEventListener('click', () => {
      const content = header.nextElementSibling;
      header.classList.toggle('collapsed');
      if (content) content.classList.toggle('hidden');
    });
  });

  // Imaging controls
  if (autoExposureToggle && displayMaterial.uniforms.autoExposureEnabled) {
    let autoExposureEnabled = Boolean(displayMaterial.uniforms.autoExposureEnabled.value);
    const updateAutoExposureToggle = () => {
      displayMaterial.uniforms.autoExposureEnabled.value = autoExposureEnabled;
      autoExposureToggle.textContent = `Auto exposure: ${autoExposureEnabled ? 'On' : 'Off'}`;
      autoExposureToggle.classList.toggle('active', autoExposureEnabled);
    };
    updateAutoExposureToggle();
    autoExposureToggle.addEventListener('click', () => {
      autoExposureEnabled = !autoExposureEnabled;
      updateAutoExposureToggle();
      autoExposureToggle.blur();
    });
  }
  if (noiseSlider) {
    displayMaterial.uniforms.noiseLevel.value = parseFloat(noiseSlider.value);
    noiseSlider.addEventListener('input', e => {
      displayMaterial.uniforms.noiseLevel.value = parseFloat(e.target.value);
    });
  }
  if (pulseRateSlider && displayMaterial.uniforms.pulseRate) {
    const updatePulseRate = e => {
      displayMaterial.uniforms.pulseRate.value = parseFloat(e.target.value);
    };
    displayMaterial.uniforms.pulseRate.value = parseFloat(pulseRateSlider.value);
    pulseRateSlider.addEventListener('input', updatePulseRate);
    pulseRateSlider.addEventListener('change', updatePulseRate);
  }
  if (scatterStrengthSlider && displayMaterial.uniforms.scatterStrength) {
    displayMaterial.uniforms.scatterStrength.value = parseFloat(scatterStrengthSlider.value);
    scatterStrengthSlider.addEventListener('input', e => {
      displayMaterial.uniforms.scatterStrength.value = parseFloat(e.target.value);
    });
  }
  if (collimationSlider && displayMaterial.uniforms.collimation) {
    displayMaterial.uniforms.collimation.value = parseFloat(collimationSlider.value);
    collimationSlider.addEventListener('input', e => {
      displayMaterial.uniforms.collimation.value = parseFloat(e.target.value);
    });
  }
  if (imageBrightnessSlider && displayMaterial.uniforms.imageBrightness) {
    displayMaterial.uniforms.imageBrightness.value = parseFloat(imageBrightnessSlider.value);
    imageBrightnessSlider.addEventListener('input', e => {
      displayMaterial.uniforms.imageBrightness.value = parseFloat(e.target.value);
    });
  }
  if (imageContrastSlider && displayMaterial.uniforms.imageContrast) {
    displayMaterial.uniforms.imageContrast.value = parseFloat(imageContrastSlider.value);
    imageContrastSlider.addEventListener('input', e => {
      displayMaterial.uniforms.imageContrast.value = parseFloat(e.target.value);
    });
  }
  if (edgeEnhancementSlider && displayMaterial.uniforms.edgeStrength) {
    displayMaterial.uniforms.edgeStrength.value = parseFloat(edgeEnhancementSlider.value);
    edgeEnhancementSlider.addEventListener('input', e => {
      displayMaterial.uniforms.edgeStrength.value = parseFloat(e.target.value);
    });
  }
  if (persistenceSlider) {
    blendMaterial.uniforms.decay.value = parseFloat(persistenceSlider.value);
    persistenceSlider.addEventListener('input', e => {
      blendMaterial.uniforms.decay.value = parseFloat(e.target.value);
    });
  }
  if (boneVisibilitySlider && displayMaterial.uniforms.boneOpacity) {
    displayMaterial.uniforms.boneOpacity.value = parseFloat(boneVisibilitySlider.value);
    boneVisibilitySlider.addEventListener('input', e => {
      displayMaterial.uniforms.boneOpacity.value = parseFloat(e.target.value);
    });
  }
  if (contrastOpacitySlider && displayMaterial.uniforms.contrastOpacity) {
    displayMaterial.uniforms.contrastOpacity.value = parseFloat(contrastOpacitySlider.value) / 100;
    contrastOpacitySlider.addEventListener('input', e => {
      displayMaterial.uniforms.contrastOpacity.value = parseFloat(e.target.value) / 100;
    });
  }
  if (contrastGainSlider && displayMaterial.uniforms.contrastGain) {
    displayMaterial.uniforms.contrastGain.value = parseFloat(contrastGainSlider.value);
    contrastGainSlider.addEventListener('input', e => {
      displayMaterial.uniforms.contrastGain.value = parseFloat(e.target.value);
    });
  }
  if (roadmapOpacitySlider && displayMaterial.uniforms.roadmapOpacity) {
    displayMaterial.uniforms.roadmapOpacity.value =
      parseFloat(roadmapOpacitySlider.value) / 100;
    roadmapOpacitySlider.addEventListener('input', e => {
      displayMaterial.uniforms.roadmapOpacity.value =
        parseFloat(e.target.value) / 100;
    });
  }
  if (
    roadmapBackgroundSlider &&
    displayMaterial.uniforms.roadmapBackgroundVisibility
  ) {
    displayMaterial.uniforms.roadmapBackgroundVisibility.value =
      parseFloat(roadmapBackgroundSlider.value) / 100;
    roadmapBackgroundSlider.addEventListener('input', e => {
      displayMaterial.uniforms.roadmapBackgroundVisibility.value =
        parseFloat(e.target.value) / 100;
    });
  }
  if (dsaGainSlider && displayMaterial.uniforms.dsaGain) {
    displayMaterial.uniforms.dsaGain.value = parseFloat(dsaGainSlider.value);
    dsaGainSlider.addEventListener('input', e => {
      displayMaterial.uniforms.dsaGain.value = parseFloat(e.target.value);
    });
  }
  acquireDsaMaskButton?.addEventListener('click', () => {
    onRequestDsaMask?.();
    acquireDsaMaskButton.blur();
  });
  toggleDsaButton?.addEventListener('click', () => {
    onToggleDsa?.();
    toggleDsaButton.blur();
  });
  captureRoadmapButton?.addEventListener('click', () => {
    onCaptureRoadmap?.();
    captureRoadmapButton.blur();
  });
  toggleRoadmapButton?.addEventListener('click', () => {
    onToggleRoadmap?.();
    toggleRoadmapButton.blur();
  });
  clearRoadmapButton?.addEventListener('click', () => {
    onClearRoadmap?.();
    clearRoadmapButton.blur();
  });
  let dsaHoldActive = false;
  const startDsaHold = () => {
    if (dsaHoldActive) return;
    dsaHoldActive = true;
    onStartDsaRecording?.();
  };
  const stopDsaHold = () => {
    if (!dsaHoldActive) return;
    dsaHoldActive = false;
    onStopDsaRecording?.();
  };
  recordDsaSequenceButton?.addEventListener('pointerdown', e => {
    startDsaHold();
    recordDsaSequenceButton.setPointerCapture?.(e.pointerId);
    e.preventDefault();
  });
  recordDsaSequenceButton?.addEventListener('pointerup', stopDsaHold);
  recordDsaSequenceButton?.addEventListener('pointercancel', stopDsaHold);
  recordDsaSequenceButton?.addEventListener('lostpointercapture', stopDsaHold);
  dsaSequenceSelect?.addEventListener('change', e => {
    if (e.target.value !== '') onSelectDsaSequence?.(Number(e.target.value));
  });
  dsaFrameSelect?.addEventListener('input', e => {
    const sequenceId = Number(dsaSequenceSelect?.value);
    if (Number.isFinite(sequenceId) && dsaSequenceSelect?.value !== '') {
      onSelectDsaFrame?.(sequenceId, Number(e.target.value));
    }
  });
  useBestDsaFrameButton?.addEventListener('click', () => {
    const sequenceId = dsaSequenceSelect?.value === ''
      ? undefined
      : Number(dsaSequenceSelect.value);
    onUseBestDsaFrame?.(sequenceId);
    useBestDsaFrameButton.blur();
  });
  dsaSequenceGalleryEl?.addEventListener('click', event => {
    const actionButton = event.target.closest('[data-dsa-gallery-action]');
    if (!actionButton || !dsaSequenceGalleryEl.contains(actionButton)) return;
    const sequenceId = Number(actionButton.dataset.sequenceId);
    if (!Number.isFinite(sequenceId)) return;
    if (actionButton.dataset.dsaGalleryAction === 'cine') {
      onToggleDsaCine?.(sequenceId);
    } else if (actionButton.dataset.dsaGalleryAction === 'roadmap') {
      onSelectDsaSequence?.(sequenceId);
    }
    actionButton.blur();
  });
  dsaCinePlayPauseButton?.addEventListener('click', () => {
    onToggleDsaCine?.();
    dsaCinePlayPauseButton.blur();
  });
  dsaCineStopButton?.addEventListener('click', () => {
    onStopDsaCine?.();
    dsaCineStopButton.blur();
  });

  // Physics controls
  if (bendSlider) {
    let bendingStiffness = parseFloat(bendSlider.value);
    setBendingStiffness(bendingStiffness);
    bendSlider.addEventListener('input', e => {
      bendingStiffness = parseFloat(e.target.value);
      setBendingStiffness(bendingStiffness);
    });
  }
  if (staticFricSlider && kineticFricSlider) {
    let staticFriction = parseFloat(staticFricSlider.value);
    let kineticFriction = parseFloat(kineticFricSlider.value);
    setWallFriction(staticFriction, kineticFriction);
    staticFricSlider.addEventListener('input', e => {
      staticFriction = parseFloat(e.target.value);
      setWallFriction(staticFriction, kineticFriction);
    });
    kineticFricSlider.addEventListener('input', e => {
      kineticFriction = parseFloat(e.target.value);
      setWallFriction(staticFriction, kineticFriction);
    });
  }
  if (smoothIterSlider) {
    let smoothingIterations = parseInt(smoothIterSlider.value);
    setSmoothingIterations(smoothingIterations);
    smoothIterSlider.addEventListener('input', e => {
      smoothingIterations = parseInt(e.target.value);
      setSmoothingIterations(smoothingIterations);
    });
  }

  // Mode toggle
  let fluoroscopy = true;
  if (modeToggle) {
    const updateModeToggle = () => {
      modeToggle.classList.toggle('fluoro-active', fluoroscopy);
      modeToggle.classList.toggle('debug-active', !fluoroscopy);
      modeToggle.setAttribute('aria-pressed', String(!fluoroscopy));
      modeToggle.setAttribute('aria-label', `Current view: ${fluoroscopy ? 'fluoroscopy' : 'debug'}`);
    };
    updateModeToggle();
    displayMaterial.uniforms.fluoroscopy.value = true;
    modeToggle.addEventListener('click', () => {
      fluoroscopy = !fluoroscopy;
      displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
      updateModeToggle();
      // Render the guidewire in white so it appears black after inversion
      if (wireMaterial) wireMaterial.color.set(0xffffff);
      if (typeof onModeChange === 'function') onModeChange(fluoroscopy);
    });
    // Set initial mode state to simulator
    if (typeof onModeChange === 'function') onModeChange(fluoroscopy);
  }

  // Keyboard controls for guidewire advance and keyboard-triggered injection
  let advance = 0;
  let catheterAdvance = 0;
  let catheterRotation = 0;
  const setCatheterAdvance = value => {
    if (value !== 0) stopCatheterAutoWithdraw();
    catheterAdvance = value;
  };
  const stopCatheterAdvance = () => {
    catheterAdvance = 0;
  };
  const setCatheterRotation = value => {
    catheterRotation = value;
  };
  const stopCatheterRotation = () => {
    catheterRotation = 0;
  };

  function wireHoldButton(button, onDown, onUp) {
    if (!button) return;
    button.addEventListener('pointerdown', e => {
      onDown();
      button.setPointerCapture?.(e.pointerId);
      e.preventDefault();
    });
    button.addEventListener('pointerup', onUp);
    button.addEventListener('pointercancel', onUp);
    button.addEventListener('pointerleave', e => {
      if (e.buttons === 0) onUp();
    });
  }
  wireHoldButton(catheterAdvanceButton, () => setCatheterAdvance(1), stopCatheterAdvance);
  wireHoldButton(catheterWithdrawButton, () => setCatheterAdvance(-1), stopCatheterAdvance);
  wireHoldButton(catheterRotateLeftButton, () => setCatheterRotation(-1), stopCatheterRotation);
  wireHoldButton(catheterRotateRightButton, () => setCatheterRotation(1), stopCatheterRotation);

  document.addEventListener('keydown', e => {
    if (e.code === 'KeyW' || e.code === 'ArrowUp') {
      stopGuidewireAutoWithdraw();
      advance = 1; e.preventDefault();
    }
    if (e.code === 'KeyS' || e.code === 'ArrowDown') {
      stopGuidewireAutoWithdraw();
      advance = -1; e.preventDefault();
    }
    if (e.code === 'KeyD') {
      stopCatheterAutoWithdraw();
      catheterAdvance = 1; e.preventDefault();
    }
    if (e.code === 'KeyA') {
      stopCatheterAutoWithdraw();
      catheterAdvance = -1; e.preventDefault();
    }
    if (e.code === 'KeyE') {
      catheterRotation = 1; e.preventDefault();
    }
    if (e.code === 'KeyQ') {
      catheterRotation = -1; e.preventDefault();
    }
    if ((e.code === 'KeyI' || e.code === 'KeyC') && fluoroscopy) {
      // Trigger injection with current UI values
      if (typeof onStartInjection === 'function') {
        const rate = parseFloat(injRateSlider.value);
        const volume = parseFloat(injVolumeSlider.value);
        const source = injSourceSelect?.value || 'sheath';
        onStartInjection({ source, rate, volume });
      }
      e.preventDefault();
    }
    if (e.code === 'KeyR' && fluoroscopy && !e.repeat) {
      startDsaHold();
      e.preventDefault();
    }
  }, true);
  document.addEventListener('keyup', e => {
    if (['KeyW', 'KeyS', 'ArrowUp', 'ArrowDown'].includes(e.code)) {
      advance = 0; e.preventDefault();
    }
    if (['KeyA', 'KeyD'].includes(e.code)) {
      catheterAdvance = 0; e.preventDefault();
    }
    if (['KeyQ', 'KeyE'].includes(e.code)) {
      catheterRotation = 0; e.preventDefault();
    }
    if (e.code === 'KeyR') {
      stopDsaHold();
      e.preventDefault();
    }
  }, true);
  window.addEventListener('blur', () => {
    advance = 0;
    catheterAdvance = 0;
    catheterRotation = 0;
    stopDsaHold();
  });

  // Injection buttons
  if (injectButton) {
    injectButton.addEventListener('click', () => {
      if (typeof onStartInjection === 'function') {
        const rate = parseFloat(injRateSlider.value);
        const volume = parseFloat(injVolumeSlider.value);
        const source = injSourceSelect?.value || 'sheath';
        onStartInjection({ source, rate, volume });
      }
    });
  }
  if (stopInjectButton) {
    stopInjectButton.addEventListener('click', () => {
      if (typeof onStopInjection === 'function') onStopInjection();
    });
  }

  // Helpers to let simulator update UI
  function updateInsertedLength(cm) {
    insertedLengthCm = Math.max(0, cm);
    guidewireAutoWithdraw.updateLength(insertedLengthCm);
    const nextTenths = Math.round(insertedLengthCm * 10);
    if (nextTenths === insertedLengthTenths) return;
    insertedLengthTenths = nextTenths;
    const display = (nextTenths / 10).toFixed(1);
    insertedLengthDisplay = display;
    if (insertedLengthEl) insertedLengthEl.textContent = `Wire ${display} cm`;
    updateToolSelectionLocks();
  }
  function updateCatheterLength(cm) {
    catheterLengthCm = Math.max(0, cm);
    catheterAutoWithdraw.updateLength(catheterLengthCm);
    const nextTenths = Math.round(catheterLengthCm * 10);
    if (nextTenths === catheterLengthTenths) return;
    catheterLengthTenths = nextTenths;
    const display = (nextTenths / 10).toFixed(1);
    catheterLengthDisplay = display;
    if (catheterLengthEl) catheterLengthEl.textContent = `Catheter ${display} cm`;
    updateToolSelectionLocks();
  }
  function updateDose(ml) {
    const nextTenths = Math.round(ml * 10);
    if (nextTenths === doseTenths) return;
    doseTenths = nextTenths;
    const display = (nextTenths / 10).toFixed(1);
    doseDisplay = display;
    if (doseDisplayEl) doseDisplayEl.textContent = `Contrast ${display} ml`;
  }
  function updateXrayTechnique(kv, ma) {
    const nextRoundedKv = Math.round(kv);
    const nextMaTenths = Math.round(ma * 10);
    if (currentKVEl && nextRoundedKv !== roundedKv) {
      kvDisplay = `${nextRoundedKv} kV`;
      currentKVEl.textContent = kvDisplay;
    }
    if (currentMAEl && nextMaTenths !== maTenths) {
      maDisplay = `${(nextMaTenths / 10).toFixed(1)} mA`;
      currentMAEl.textContent = maDisplay;
    }
    roundedKv = nextRoundedKv;
    maTenths = nextMaTenths;
  }
  function updateGuidewireResistance(level, reason = '') {
    if (!guidewireResistanceEl) return;
    const percent = Math.round(Math.max(0, Math.min(1, level)) * 100);
    const moderate = level >= 0.35;
    const strong = level > 0.72;
    const displayReason = reason || 'Swobodne wsuwanie prowadnika';
    if (resistanceModerate !== moderate) {
      guidewireResistanceEl.classList.toggle('moderate', moderate);
      resistanceModerate = moderate;
    }
    if (resistanceStrong !== strong) {
      guidewireResistanceEl.classList.toggle('strong', strong);
      resistanceStrong = strong;
    }
    if (guidewireResistanceReasonEl && resistanceReason !== displayReason) {
      guidewireResistanceReasonEl.textContent = displayReason;
      resistanceReason = displayReason;
    }
    if (resistancePercent !== percent) {
      if (guidewireResistanceValueEl) guidewireResistanceValueEl.textContent = `${percent}%`;
      if (guidewireResistanceFillEl) guidewireResistanceFillEl.style.width = `${percent}%`;
      resistancePercent = percent;
    }
  }
  function formatDebugDistance(value) {
    if (!Number.isFinite(value)) return '--';
    return Math.abs(value) < 10 ? value.toFixed(2) : value.toFixed(1);
  }
  function formatDebugMs(value) {
    if (!Number.isFinite(value)) return '--';
    return value < 10 ? value.toFixed(2) : value.toFixed(1);
  }
  function formatGuidewirePerformance(perf) {
    if (!perf) return '';
    const penetrationText = Number.isFinite(perf.settledPenetration)
      ? ` | pen ${formatDebugDistance(perf.settledPenetration)}` +
        `/${formatDebugDistance(perf.maximumPenetration)} mm`
      : '';
    return `\nXPBD: adv ${formatDebugMs(perf.advanceMs)} ` +
      `/ solve ${formatDebugMs(perf.solveMs)} ` +
      `/ narrow ${formatDebugMs(perf.projectMs)} ` +
      `/ dbg ${formatDebugMs(perf.diagnosticMs)} ms | ` +
      `q ${perf.pointContactCount}+${perf.diagnosticPointContactCount} | ` +
      `segS ${perf.segmentSampleCount}` +
      `${Number.isFinite(perf.activeBranchCount) ? ` | br ${perf.activeBranchCount}` : ''}` +
      penetrationText +
      `${perf.foldGuarded ? ' | fold' : ''}` +
      `${perf.stabilityRepaired ? ' | repair' : ''}` +
      `${perf.withdrawalRelaxed ? ' | withdraw' : ''}`;
  }
  function updateGuidewireDiagnostics(metrics = null) {
    if (!guidewireDiagnosticsEl) return;
    guidewireDiagnosticsEl.classList.remove('warn', 'breach');
    if (!metrics) {
      guidewireDiagnosticsEl.textContent = 'GW STL: debug off';
      return;
    }
    const perfText = formatGuidewirePerformance(metrics.performance);
    if (!metrics.checkedCount || !Number.isFinite(metrics.minSignedDistance)) {
      guidewireDiagnosticsEl.textContent = `GW STL: no lumen samples${perfText}`;
      return;
    }

    guidewireDiagnosticsEl.classList.toggle('breach', metrics.outsideCount > 0);
    guidewireDiagnosticsEl.classList.toggle(
      'warn',
      metrics.outsideCount === 0 && metrics.clearanceViolationCount > 0
    );
    guidewireDiagnosticsEl.textContent =
      `GW STL: min ${formatDebugDistance(metrics.minSignedDistance)} mm ` +
      `/ clr ${formatDebugDistance(metrics.clearance)} | ` +
      `out ${metrics.outsideCount} | near ${metrics.clearanceViolationCount} | ` +
      `seg ${formatDebugDistance(metrics.maxSegmentError)} | ` +
      `bend ${formatDebugDistance(metrics.maxBendAngle)} deg` +
      perfText;
  }
  function setInjectButtonDisabled(disabled) {
    if (injectButton && injectButton.disabled !== !!disabled) injectButton.disabled = !!disabled;
  }
  function setStopInjectionDisabled(disabled) {
    if (stopInjectButton && stopInjectButton.disabled !== !!disabled) stopInjectButton.disabled = !!disabled;
  }
  function setInjectionSourceStatus(valid, message) {
    if (!injectionSourceStatusEl) return;
    injectionSourceStatusEl.textContent = message || (valid ? 'Ready' : 'Unavailable');
    injectionSourceStatusEl.classList.toggle('locked', !valid);
  }
  function applyInjectionRateMaximum(preview) {
    const physicalMaximum =
      Number(preview?.maximumAchievableRateMlPerSec);
    if (
      !injRateSlider ||
      !(Number.isFinite(physicalMaximum) && physicalMaximum > 0)
    ) return false;
    const minimum = parseFloat(injRateSlider.min || '0.5');
    const step = Math.max(
      1e-6,
      parseFloat(injRateSlider.step || '0.1')
    );
    const stepText = String(injRateSlider.step || '0.1');
    const decimalIndex = stepText.indexOf('.');
    const decimalPlaces = decimalIndex >= 0
      ? stepText.length - decimalIndex - 1
      : 0;
    const selectableMaximum = Math.max(
      minimum,
      Math.floor((physicalMaximum + 1e-9) / step) * step
    );
    const maximumText = selectableMaximum.toFixed(decimalPlaces);
    const currentRateBeforeMaximumChange = parseFloat(
      injRateSlider.value || '0'
    );
    if (injRateSlider.max !== maximumText) {
      injRateSlider.max = maximumText;
    }
    if (
      currentRateBeforeMaximumChange <= selectableMaximum + 1e-9
    ) return false;
    injRateSlider.value = maximumText;
    const valueLabel = injRateSlider.nextElementSibling;
    if (valueLabel) valueLabel.textContent = maximumText;
    updateInjectionDuration();
    return true;
  }
  function updateInjectionHydraulics(preview) {
    lastInjectionHydraulicPreview = preview || null;
    applyInjectionRateMaximum(preview);
    const valid = !!preview?.valid;
    injectionHydraulicSummaryEl?.classList.toggle(
      'pressure-limited',
      valid && preview.pressureLimited
    );
    if (!valid) {
      if (injActualRateOutput) injActualRateOutput.textContent = '— ml/s';
      if (injPressureOutput) injPressureOutput.textContent = '— psi';
      if (injPressureWarningOutput) {
        injPressureWarningOutput.textContent =
          preview?.reason || 'Injection source unavailable';
      }
      return;
    }
    if (injActualRateOutput) {
      injActualRateOutput.textContent =
        `${preview.actualRateMlPerSec.toFixed(1)} ml/s`;
    }
    if (injPressureOutput) {
      injPressureOutput.textContent = preview.pressureLimited
        ? `${preview.appliedPressurePsi.toFixed(0)} / ` +
          `${preview.requiredPressurePsi.toFixed(0)} psi`
        : `${preview.appliedPressurePsi.toFixed(0)} psi`;
    }
    if (injPressureWarningOutput) {
      injPressureWarningOutput.textContent = preview.pressureLimited
        ? `Limited by ${preview.limitingComponent}: requested ` +
          `${preview.requestedRateMlPerSec.toFixed(1)}, achievable ` +
          `${preview.actualRateMlPerSec.toFixed(1)} ml/s at ` +
          `${preview.pressureLimitPsi.toFixed(0)} psi`
        : `${preview.deviceLabel}: requested rate is achievable · ` +
          `max ${preview.maximumAchievableRateMlPerSec.toFixed(1)} ml/s · ` +
          `${preview.outletCount} outlet${preview.outletCount === 1 ? '' : 's'}`;
    }
    if (injDurationOutput) {
      const volume = parseFloat(injVolumeSlider?.value || '0');
      const duration = volume / Math.max(
        1e-9,
        preview.actualRateMlPerSec
      );
      injDurationOutput.value = `${duration.toFixed(2)} s`;
      injDurationOutput.textContent = injDurationOutput.value;
    }
  }
  function updateContrastDiagnostics(metrics) {
    if (!contrastDiagnosticsEl) return;
    if (!metrics) {
      contrastDiagnosticsEl.textContent = 'Contrast: model loading';
      contrastDiagnosticsEl.classList.remove('warn');
      return;
    }
    const balancePercent = Math.abs(metrics.relativeBalanceError) * 100;
    const column = metrics.retrogradeColumn;
    const flowSplit = metrics.continuousFlowSplit;
    const hydraulics = metrics.injectionHydraulics;
    const reverseFlow = flowSplit?.active
      ? `${flowSplit.upstreamRateMlPerSec.toFixed(1)}↑/` +
        `${flowSplit.downstreamRateMlPerSec.toFixed(1)}↓ ml/s · ` +
        `${flowSplit.reversedEdgeCount} reversed · ` +
        `min ${flowSplit.minimumSignedFlowMlPerSec.toFixed(1)} ml/s`
      : 'off';
    const injectionSource = metrics.lastSourceEdgeIndex >= 0
      ? `${metrics.lastSourceEdgeIndex}` +
        ` t${metrics.lastSourceT.toFixed(2)}` +
        ` r${metrics.lastSourceRadiusMm.toFixed(1)}` +
        ` d${metrics.lastSourceDistanceMm.toFixed(1)}` +
        ` dir${metrics.lastDirectionAgainstFlow.toFixed(2)}` +
        ` J${metrics.lastRetrogradeMomentumFluxRatio.toFixed(1)}` +
        ` ${metrics.lastSourceSelectionMode}` +
        ` Δ${metrics.sourceMappingChangeCount}`
      : 'none';
    const hydraulicText = hydraulics
      ? `${hydraulics.requestedRateMlPerSec.toFixed(1)}→` +
        `${hydraulics.actualRateMlPerSec.toFixed(1)} ml/s · ` +
        `${hydraulics.appliedPressurePsi.toFixed(0)}/` +
        `${hydraulics.pressureLimitPsi.toFixed(0)} psi · ` +
        `${hydraulics.flowRegime}`
      : 'idle';
    const jetText = metrics.lastPhysicalJetSpeedMmPerSec > 0
      ? `${(metrics.lastPhysicalJetSpeedMmPerSec / 1000).toFixed(1)}→` +
        `${(metrics.lastResolvedJetSpeedMmPerSec / 1000).toFixed(1)} m/s · ` +
        `mix ${metrics.lastJetMixingLengthMm.toFixed(1)} mm · ` +
        `core ${(metrics.lastTargetedCoreFraction * 100).toFixed(0)}%`
      : 'idle';
    contrastDiagnosticsEl.classList.toggle(
      'warn',
      balancePercent > 0.5 || !!hydraulics?.pressureLimited
    );
    contrastDiagnosticsEl.textContent =
      `Contrast: ${metrics.intravascularIodineMassMg.toFixed(1)} mg vascular · ` +
      `${metrics.localIodineMassMg.toFixed(1)} mg local · ` +
      `${metrics.outletIodineMassMg.toFixed(1)} mg out · ` +
      `${metrics.activeParticleCount} parcels · ` +
      `hyd ${hydraulicText} · ` +
      `jet ${jetText} · ` +
      `reverse ${reverseFlow} · ` +
      `column ${(column.maximumContiguousFilledFraction * 100).toFixed(0)}% · ` +
      `aorta ${column.totalAorticHandoffIodineMassMg.toFixed(1)} mg · ` +
      `source ${injectionSource} · ` +
      `balance ${balancePercent.toFixed(3)}%`;
  }
  let perfElapsed = 0;
  let perfFrames = 0;
  function updatePerfStats(dtSeconds) {
    if (!perfStatsEl) return;
    perfElapsed += dtSeconds;
    perfFrames++;
    if (perfElapsed < 0.25) return;
    const fps = (perfFrames / Math.max(1e-6, perfElapsed)).toFixed(1);
    let mem = 'N/A';
    if (performance.memory) {
      mem = (performance.memory.usedJSHeapSize / 1048576).toFixed(1) + ' MB';
    }
    perfStatsEl.textContent = `FPS: ${fps} | Mem: ${mem}`;
    perfElapsed = 0;
    perfFrames = 0;
  }
  function updateBrowserBenchmarkStatus(status, report = null) {
    const running = !!status?.running;
    if (runBrowserBenchmarkSmokeButton) runBrowserBenchmarkSmokeButton.disabled = running;
    if (runBrowserBenchmarkFullButton) runBrowserBenchmarkFullButton.disabled = running;
    if (stopBrowserBenchmarkButton) stopBrowserBenchmarkButton.disabled = !running;
    if (!browserBenchmarkStatusEl) return;

    browserBenchmarkStatusEl.classList.remove('passed', 'failed');
    if (running) {
      if (browserBenchmarkReportEl) browserBenchmarkReportEl.value = 'Running';
      if (status.warmingUp) {
        browserBenchmarkStatusEl.textContent = 'Warming up';
        return;
      }
      const elapsedSeconds = Math.floor(status.elapsedMs / 1000);
      const durationSeconds = Math.round(status.durationMs / 1000);
      browserBenchmarkStatusEl.textContent =
        `Running ${elapsedSeconds}/${durationSeconds} s · cycle ${status.cycleIndex + 1}`;
      return;
    }
    if (!report?.frameCount) {
      browserBenchmarkStatusEl.textContent = 'Idle';
      if (browserBenchmarkReportEl) browserBenchmarkReportEl.value = 'No report';
      return;
    }

    const acceptance = report.browserAcceptance;
    const fullRun = status.durationMs >= 600000 && status.elapsedMs >= 600000;
    const passed = fullRun && !!acceptance?.passed;
    browserBenchmarkStatusEl.classList.add(passed ? 'passed' : 'failed');
    browserBenchmarkStatusEl.textContent =
      `${fullRun ? (passed ? 'PASS' : 'FAIL') : 'Smoke'} · ` +
      `${report.averageFps.toFixed(1)} FPS · 1% ${report.onePercentLowFps.toFixed(1)} · ` +
      `pen ${report.physicsEnvelope.maxPostStepPenetrationMm.toFixed(3)} mm`;
    if (browserBenchmarkReportEl) browserBenchmarkReportEl.value = JSON.stringify(report);
  }

  function updateCatheterAortaSetupStatus(status) {
    const running = !!status?.running;
    if (prepareCatheterAortaButton) prepareCatheterAortaButton.disabled = running;
    if (reproduceIliacContrastBugButton) reproduceIliacContrastBugButton.disabled = running;
    if (reproduceRetrogradeGapButton) reproduceRetrogradeGapButton.disabled = running;
    if (reproduceArchBolusButton) reproduceArchBolusButton.disabled = running;
    if (!catheterAortaSetupStatusEl) return;

    if (status?.phase === 'guidewire') {
      catheterAortaSetupStatusEl.textContent =
        `Wsuwanie prowadnika do aorty · ${status.guidewireProgressCm.toFixed(1)}/${status.guidewireTargetCm.toFixed(1)} cm`;
      return;
    }
    if (status?.phase === 'catheter') {
      catheterAortaSetupStatusEl.textContent =
        `Nasuwanie cewnika po prowadniku · ${status.catheterProgressCm.toFixed(1)}/${status.catheterTargetCm.toFixed(1)} cm`;
      return;
    }
    if (status?.phase === 'guidewire-withdraw') {
      catheterAortaSetupStatusEl.textContent =
        `Wycofywanie prowadnika · ${status.guidewireProgressCm.toFixed(1)}/${status.finalGuidewireTargetCm.toFixed(1)} cm`;
      return;
    }
    if (status?.phase === 'ready') {
      catheterAortaSetupStatusEl.textContent =
        `Cewnik ${status.catheterProgressCm.toFixed(1)} cm ustawiony · prowadnik ${status.guidewireProgressCm.toFixed(1)} cm pozostawiony`;
      return;
    }
    catheterAortaSetupStatusEl.textContent = 'Gotowe do ustawienia';
  }

  function setAutomatedBenchmarkMode(enabled) {
    const active = enabled === true;
    if (active) cArmControls?.reset?.();
    cArmControls?.setLocked?.(active);
    document.body.classList.toggle('automated-benchmark-running', active);
  }

  let dsaSequenceOptionSignature = '';
  let dsaGallerySignature = '';

  function updateDsaRoadmapState(state = {}) {
    const maskPending = state.maskCapturePending === true;
    const roadmapPending = state.roadmapCapturePending === true;
    const maskValid = state.maskValid === true;
    const dsaEnabled = state.dsaEnabled === true;
    const roadmapValid = state.roadmapValid === true;
    const roadmapEnabled = state.roadmapEnabled === true;
    const recording = state.recording === true;
    const preparingRecording = recording && maskPending;
    const completedSequences = (state.sequences || []).filter(
      sequence => sequence.complete && sequence.frames?.length
    );
    const cineSequence = completedSequences.find(
      sequence => sequence.id === state.cineSequenceId
    ) || null;
    const cineActive = !!cineSequence && Number.isInteger(state.cineFrameIndex);

    if (acquireDsaMaskButton) {
      acquireDsaMaskButton.disabled = maskPending || roadmapPending || recording;
      acquireDsaMaskButton.textContent = maskPending
        ? 'Acquiring…'
        : maskValid
          ? 'Reacquire mask'
          : 'Acquire mask';
    }
    if (toggleDsaButton) {
      toggleDsaButton.disabled = !maskValid || maskPending || roadmapPending || recording;
      toggleDsaButton.textContent = `DSA: ${dsaEnabled ? 'On' : 'Off'}`;
      toggleDsaButton.classList.toggle('active', dsaEnabled);
      toggleDsaButton.setAttribute('aria-pressed', String(dsaEnabled));
    }
    if (recordDsaSequenceButton) {
      recordDsaSequenceButton.textContent = preparingRecording
        ? 'Preparing C-arm… keep holding R'
        : recording
          ? 'Recording… release R'
          : 'Hold R: Record';
      recordDsaSequenceButton.classList.toggle('recording', recording);
      recordDsaSequenceButton.classList.toggle('preparing', preparingRecording);
      recordDsaSequenceButton.setAttribute('aria-pressed', String(recording));
    }
    if (captureRoadmapButton) {
      captureRoadmapButton.disabled = !maskValid || maskPending || roadmapPending || recording;
      captureRoadmapButton.textContent = roadmapPending
        ? 'Capturing…'
        : 'Use current DSA';
    }
    if (toggleRoadmapButton) {
      toggleRoadmapButton.disabled = !roadmapValid || roadmapPending || recording;
      toggleRoadmapButton.textContent =
        `Roadmap: ${roadmapEnabled ? 'On' : 'Off'}`;
      toggleRoadmapButton.classList.toggle('active', roadmapEnabled);
      toggleRoadmapButton.setAttribute('aria-pressed', String(roadmapEnabled));
    }
    if (clearRoadmapButton) {
      clearRoadmapButton.disabled = !roadmapValid || roadmapPending || recording;
    }
    const optionSignature = completedSequences
      .map(sequence => `${sequence.id}:${sequence.frames.length}:${sequence.bestFrameIndex}`)
      .join('|');
    if (dsaSequenceSelect && optionSignature !== dsaSequenceOptionSignature) {
      dsaSequenceOptionSignature = optionSignature;
      dsaSequenceSelect.replaceChildren();
      if (!completedSequences.length) {
        const option = document.createElement('option');
        option.value = '';
        option.textContent = 'No saved sequence';
        dsaSequenceSelect.append(option);
      } else {
        for (const sequence of completedSequences) {
          const option = document.createElement('option');
          option.value = String(sequence.id);
          option.textContent =
            `DSA ${sequence.id} · ${sequence.frames.length} frames`;
          dsaSequenceSelect.append(option);
        }
      }
    }
    const selectedSequence = completedSequences.find(
      sequence => sequence.id === state.selectedSequenceId
    ) || completedSequences.at(-1) || null;
    if (dsaSequenceSelect) {
      dsaSequenceSelect.disabled = recording || !completedSequences.length;
      dsaSequenceSelect.value = selectedSequence
        ? String(selectedSequence.id)
        : '';
    }
    const selectedFrameIndex = selectedSequence
      ? state.selectedSequenceId === selectedSequence.id &&
        Number.isInteger(state.selectedFrameIndex)
          ? state.selectedFrameIndex
          : selectedSequence.selectedFrameIndex ??
            selectedSequence.bestFrameIndex ?? 0
      : 0;
    if (dsaFrameSelect) {
      dsaFrameSelect.disabled = recording || !selectedSequence;
      dsaFrameSelect.min = '0';
      dsaFrameSelect.max = String(
        Math.max(0, (selectedSequence?.frames.length || 1) - 1)
      );
      dsaFrameSelect.value = String(selectedFrameIndex);
    }
    if (useBestDsaFrameButton) {
      useBestDsaFrameButton.disabled = recording || !selectedSequence;
    }
    if (dsaFrameInfoEl) {
      const frame = selectedSequence?.frames[selectedFrameIndex];
      const isBest = selectedSequence &&
        selectedFrameIndex === selectedSequence.bestFrameIndex;
      dsaFrameInfoEl.textContent = frame
        ? `Frame ${selectedFrameIndex + 1}/${selectedSequence.frames.length} · contrast score ${(frame.contrastScore * 100).toFixed(2)}${isBest ? ' · best-filled' : ''}`
        : recording
          ? 'Recording DSA frames…'
          : 'Hold R to record a sequence';
    }
    const gallerySignature = completedSequences
      .map(sequence => [
        sequence.id,
        sequence.frames.length,
        sequence.bestFrameIndex,
        sequence.previewKey || '',
        state.cineSequenceId === sequence.id ? state.cinePlaying : false,
        state.selectedSequenceId === sequence.id
      ].join(':'))
      .join('|');
    if (dsaSequenceGalleryEl && gallerySignature !== dsaGallerySignature) {
      dsaGallerySignature = gallerySignature;
      dsaSequenceGalleryEl.replaceChildren();
      if (!completedSequences.length) {
        const empty = document.createElement('div');
        empty.className = 'dsa-gallery-empty';
        empty.textContent = 'Recorded angiography will appear here';
        dsaSequenceGalleryEl.append(empty);
      } else {
        for (const sequence of [...completedSequences].reverse()) {
          const card = document.createElement('article');
          card.className = 'dsa-gallery-card';
          card.classList.toggle(
            'cine-active',
            state.cineSequenceId === sequence.id
          );
          card.classList.toggle(
            'roadmap-selected',
            state.selectedSequenceId === sequence.id
          );

          const preview = document.createElement('button');
          preview.type = 'button';
          preview.className = 'dsa-gallery-preview';
          preview.dataset.dsaGalleryAction = 'cine';
          preview.dataset.sequenceId = String(sequence.id);
          preview.title = `Play DSA ${sequence.id} as cine`;
          if (sequence.previewUrl) {
            const image = document.createElement('img');
            image.src = sequence.previewUrl;
            image.alt = `Best-filled frame from DSA ${sequence.id}`;
            preview.append(image);
          } else {
            preview.textContent = `DSA ${sequence.id}`;
          }

          const firstFrameTime = sequence.frames[0]?.capturedAtMs || 0;
          const lastFrameTime = sequence.frames.at(-1)?.capturedAtMs || firstFrameTime;
          const durationSeconds = Math.max(0, lastFrameTime - firstFrameTime) / 1000;
          const meta = document.createElement('div');
          meta.className = 'dsa-gallery-meta';
          const name = document.createElement('strong');
          name.textContent = `DSA ${sequence.id}`;
          const details = document.createElement('span');
          details.textContent = `${sequence.frames.length} fr · ${durationSeconds.toFixed(1)} s`;
          meta.append(name, details);

          const actions = document.createElement('div');
          actions.className = 'dsa-gallery-actions';
          const cineButton = document.createElement('button');
          cineButton.type = 'button';
          cineButton.dataset.dsaGalleryAction = 'cine';
          cineButton.dataset.sequenceId = String(sequence.id);
          cineButton.textContent =
            state.cineSequenceId === sequence.id && state.cinePlaying
              ? 'Pause cine'
              : 'Play cine';
          const roadmapButton = document.createElement('button');
          roadmapButton.type = 'button';
          roadmapButton.dataset.dsaGalleryAction = 'roadmap';
          roadmapButton.dataset.sequenceId = String(sequence.id);
          roadmapButton.textContent = 'Use roadmap';
          actions.append(cineButton, roadmapButton);
          card.append(preview, meta, actions);
          dsaSequenceGalleryEl.append(card);
        }
      }
    }
    if (dsaCineControlsEl) dsaCineControlsEl.hidden = !cineActive;
    if (dsaCineStatusEl) {
      dsaCineStatusEl.textContent = cineActive
        ? `CINE DSA ${cineSequence.id} · ${state.cineFrameIndex + 1}/${cineSequence.frames.length}`
        : 'CINE';
    }
    if (dsaCinePlayPauseButton) {
      dsaCinePlayPauseButton.textContent = state.cinePlaying ? 'Pause' : 'Play';
    }
    if (dsaCineStopButton) dsaCineStopButton.disabled = !cineActive;
    if (dsaRoadmapStatusEl) {
      dsaRoadmapStatusEl.textContent = state.status || 'DSA ready';
      dsaRoadmapStatusEl.classList.toggle(
        'warn',
        /wait|changed|failed|first|inject|unavailable|no dsa frame/i.test(state.status || '')
      );
    }
    if (imagingModeBadgeEl) {
      const activeLabel = cineActive
        ? `CINE DSA ${cineSequence.id}`
        : recording
        ? 'DSA REC'
        : dsaEnabled
        ? 'DSA'
        : roadmapEnabled
          ? 'ROADMAP'
          : '';
      imagingModeBadgeEl.hidden = !activeLabel;
      imagingModeBadgeEl.textContent = activeLabel;
      imagingModeBadgeEl.classList.toggle(
        'roadmap-active',
        roadmapEnabled && !dsaEnabled && !recording && !cineActive
      );
      imagingModeBadgeEl.classList.toggle('cine-active', cineActive);
    }
  }

  return {
    monitor,
    getAdvance: () => guidewireAutoWithdraw.active ? guidewireAutoWithdraw.command : advance,
    getCatheterAdvance: () => catheterAutoWithdraw.active ? catheterAutoWithdraw.command : catheterAdvance,
    getCatheterRotation: () => catheterRotation,
    getSelectedCatheterType: () => selectedCatheterType,
    getSelectedGuidewireType: () => selectedGuidewireType,
    getInjectionSource: () => injSourceSelect?.value || 'sheath',
    getInjectionRequest: () => ({
      source: injSourceSelect?.value || 'sheath',
      rateMlPerSec: parseFloat(injRateSlider?.value || '0'),
      volumeMl: parseFloat(injVolumeSlider?.value || '0')
    }),
    getFluoroscopy: () => fluoroscopy,
    getDebugLayerState: () => ({ ...debugLayerState }),
    updateInsertedLength,
    updateCatheterLength,
    updateDose,
    updateXrayTechnique,
    updateGuidewireResistance,
    updateGuidewireDiagnostics,
    setInjectButtonDisabled,
    setStopInjectionDisabled,
    setInjectionSourceStatus,
    updateInjectionHydraulics,
    updateContrastDiagnostics,
    updatePerfStats,
    updateCatheterAortaSetupStatus,
    updateBrowserBenchmarkStatus,
    updateDsaRoadmapState,
    setAutomatedBenchmarkMode,
    getCArmRevision: () => cArmControls?.getRevision?.() ?? 0,
  };
}
