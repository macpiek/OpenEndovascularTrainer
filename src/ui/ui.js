import { PatientMonitor } from './patientMonitor.js?v=20260721performance1';
import { initCArmPreview, renderCArmPreview, cArmPreviewGroup, cArmPreviewGantry, cArmPreviewDetectorAssembly, cArmPreviewTable } from './carmPreview.js?v=20260620rollpreview1';
import { setupCArmControls } from '../carmControls.js?v=20260620rollpreview1';
import { setBendingStiffness, setWallFriction, setSmoothingIterations } from '../physics/elasticRod.js?v=20260614carmaxis3';
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
    onStartBrowserBenchmark,
    onStopBrowserBenchmark,
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
  const injectButton = document.getElementById('injectContrast');
  const stopInjectButton = document.getElementById('stopInjection');
  const injRateSlider = document.getElementById('injRate');
  const injDurationSlider = document.getElementById('injDuration');
  const injVolumeSlider = document.getElementById('injVolume');
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
  const insertedLengthEl = document.getElementById('insertedLength');
  const catheterLengthEl = document.getElementById('catheterLength');
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
  const guidewireDiameterEl = document.getElementById('guidewireDiameter');
  const sheathDiameterEl = document.getElementById('sheathDiameter');
  const catheterDiameterEl = document.getElementById('catheterDiameter');
  const perfStatsEl = document.getElementById('perfStats');
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
  let resistanceVisible = null;
  let resistanceStrong = null;
  let resistancePercent = -1;
  let resistanceReason = '';
  let selectedCatheterType = catheterTypeSelect?.value || 'pigtail';
  let selectedGuidewireType = guidewireTypeSelect?.value || 'glidewire';
  const TOOL_SELECTION_UNLOCK_EPSILON_CM = 0.05;

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
  }

  catheterTypeSelect?.addEventListener('change', e => {
    selectedCatheterType = e.target.value;
  });
  guidewireTypeSelect?.addEventListener('change', e => {
    selectedGuidewireType = e.target.value;
  });
  updateToolSelectionLocks();

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
    injDurationSlider
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
      advance = 1; e.preventDefault();
    }
    if (e.code === 'KeyS' || e.code === 'ArrowDown') {
      advance = -1; e.preventDefault();
    }
    if (e.code === 'KeyD') {
      catheterAdvance = 1; e.preventDefault();
    }
    if (e.code === 'KeyA') {
      catheterAdvance = -1; e.preventDefault();
    }
    if (e.code === 'KeyE') {
      catheterRotation = 1; e.preventDefault();
    }
    if (e.code === 'KeyQ') {
      catheterRotation = -1; e.preventDefault();
    }
    if (e.code === 'KeyC' && fluoroscopy) {
      // Trigger injection with current UI values
      if (typeof onStartInjection === 'function') {
        const rate = parseFloat(injRateSlider.value);
        const duration = parseFloat(injDurationSlider.value) / 1000;
        const volume = parseFloat(injVolumeSlider.value);
        onStartInjection({ rate, duration, volume });
      }
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
  }, true);
  window.addEventListener('blur', () => {
    advance = 0;
    catheterAdvance = 0;
    catheterRotation = 0;
  });

  // Injection buttons
  if (injectButton) {
    injectButton.addEventListener('click', () => {
      if (typeof onStartInjection === 'function') {
        const rate = parseFloat(injRateSlider.value);
        const duration = parseFloat(injDurationSlider.value) / 1000;
        const volume = parseFloat(injVolumeSlider.value);
        onStartInjection({ rate, duration, volume });
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
    if (level < 0.35) {
      if (resistanceVisible !== false) {
        guidewireResistanceEl.classList.add('hidden');
        guidewireResistanceEl.classList.remove('strong');
        if (guidewireResistanceReasonEl) guidewireResistanceReasonEl.textContent = 'Opór na prowadniku';
        if (guidewireResistanceValueEl) guidewireResistanceValueEl.textContent = '0%';
        if (guidewireResistanceFillEl) guidewireResistanceFillEl.style.width = '0%';
        resistanceVisible = false;
        resistanceStrong = false;
        resistancePercent = 0;
        resistanceReason = '';
      }
      return;
    }
    const percent = Math.round(Math.max(0, Math.min(1, level)) * 100);
    const strong = level > 0.72;
    const displayReason = reason || 'Opór na prowadniku - cofnij lekko lub zmień kierunek.';
    if (resistanceVisible !== true) {
      guidewireResistanceEl.classList.remove('hidden');
      resistanceVisible = true;
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

  function setAutomatedBenchmarkMode(enabled) {
    const active = enabled === true;
    if (active) cArmControls?.reset?.();
    cArmControls?.setLocked?.(active);
    document.body.classList.toggle('automated-benchmark-running', active);
  }

  return {
    monitor,
    getAdvance: () => advance,
    getCatheterAdvance: () => catheterAdvance,
    getCatheterRotation: () => catheterRotation,
    getSelectedCatheterType: () => selectedCatheterType,
    getSelectedGuidewireType: () => selectedGuidewireType,
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
    updatePerfStats,
    updateBrowserBenchmarkStatus,
    setAutomatedBenchmarkMode,
    getCArmRevision: () => cArmControls?.getRevision?.() ?? 0,
  };
}
