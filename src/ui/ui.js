import { PatientMonitor } from './patientMonitor.js?v=20260614carmaxis3';
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
  setupCArmControls(
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
    if (insertedLengthEl) insertedLengthEl.textContent = 'Wire ' + cm.toFixed(1) + ' cm';
  }
  function updateCatheterLength(cm) {
    if (catheterLengthEl) catheterLengthEl.textContent = 'Catheter ' + cm.toFixed(1) + ' cm';
  }
  function updateDose(ml) {
    if (doseDisplayEl) doseDisplayEl.textContent = ml.toFixed(1) + ' ml';
  }
  function updateXrayTechnique(kv, ma) {
    if (currentKVEl) currentKVEl.textContent = `${Math.round(kv)} kV`;
    if (currentMAEl) currentMAEl.textContent = `${ma.toFixed(1)} mA`;
  }
  function updateGuidewireResistance(level, reason = '') {
    if (!guidewireResistanceEl) return;
    if (level < 0.35) {
      guidewireResistanceEl.classList.add('hidden');
      guidewireResistanceEl.classList.remove('strong');
      if (guidewireResistanceReasonEl) guidewireResistanceReasonEl.textContent = 'Opór na prowadniku';
      if (guidewireResistanceValueEl) guidewireResistanceValueEl.textContent = '0%';
      if (guidewireResistanceFillEl) guidewireResistanceFillEl.style.width = '0%';
      return;
    }
    const percent = Math.round(Math.max(0, Math.min(1, level)) * 100);
    guidewireResistanceEl.classList.remove('hidden');
    guidewireResistanceEl.classList.toggle('strong', level > 0.72);
    if (guidewireResistanceReasonEl) {
      guidewireResistanceReasonEl.textContent = reason || 'Opór na prowadniku - cofnij lekko lub zmień kierunek.';
    }
    if (guidewireResistanceValueEl) guidewireResistanceValueEl.textContent = `${percent}%`;
    if (guidewireResistanceFillEl) guidewireResistanceFillEl.style.width = `${percent}%`;
  }
  function formatDebugDistance(value) {
    if (!Number.isFinite(value)) return '--';
    return Math.abs(value) < 10 ? value.toFixed(2) : value.toFixed(1);
  }
  function formatDebugMs(value) {
    if (!Number.isFinite(value)) return '--';
    return value < 10 ? value.toFixed(2) : value.toFixed(1);
  }
  function updateGuidewireDiagnostics(metrics = null) {
    if (!guidewireDiagnosticsEl) return;
    guidewireDiagnosticsEl.classList.remove('warn', 'breach');
    if (!metrics) {
      guidewireDiagnosticsEl.textContent = 'GW STL: debug off';
      return;
    }
    if (!metrics.checkedCount || !Number.isFinite(metrics.minSignedDistance)) {
      guidewireDiagnosticsEl.textContent = 'GW STL: no lumen samples';
      return;
    }

    guidewireDiagnosticsEl.classList.toggle('breach', metrics.outsideCount > 0);
    guidewireDiagnosticsEl.classList.toggle(
      'warn',
      metrics.outsideCount === 0 && metrics.clearanceViolationCount > 0
    );
    const perf = metrics.performance;
    const perfText = perf
      ? `\nGW perf: adv ${formatDebugMs(perf.advanceMs)} ` +
        `/ solve ${formatDebugMs(perf.solveMs)} ` +
        `/ proj ${formatDebugMs(perf.projectMs)} ` +
        `/ dbg ${formatDebugMs(perf.diagnosticMs)} ms | ` +
        `q ${perf.pointContactCount}+${perf.diagnosticPointContactCount} | ` +
        `segS ${perf.segmentSampleCount}` +
        `${perf.foldGuarded ? ' | fold' : ''}` +
        `${perf.stabilityRepaired ? ' | repair' : ''}` +
        `${perf.withdrawalRelaxed ? ' | withdraw' : ''}`
      : '';
    guidewireDiagnosticsEl.textContent =
      `GW STL: min ${formatDebugDistance(metrics.minSignedDistance)} mm ` +
      `/ clr ${formatDebugDistance(metrics.clearance)} | ` +
      `out ${metrics.outsideCount} | near ${metrics.clearanceViolationCount} | ` +
      `seg ${formatDebugDistance(metrics.maxSegmentError)} | ` +
      `bend ${formatDebugDistance(metrics.maxBendAngle)} deg` +
      perfText;
  }
  function setInjectButtonDisabled(disabled) {
    if (injectButton) injectButton.disabled = !!disabled;
  }
  function setStopInjectionDisabled(disabled) {
    if (stopInjectButton) stopInjectButton.disabled = !!disabled;
  }
  function updatePerfStats(dtSeconds) {
    if (!perfStatsEl) return;
    const fps = (1 / dtSeconds).toFixed(1);
    let mem = 'N/A';
    if (performance.memory) {
      mem = (performance.memory.usedJSHeapSize / 1048576).toFixed(1) + ' MB';
    }
    perfStatsEl.textContent = `FPS: ${fps} | Mem: ${mem}`;
  }

  return {
    monitor,
    getAdvance: () => advance,
    getCatheterAdvance: () => catheterAdvance,
    getCatheterRotation: () => catheterRotation,
    getFluoroscopy: () => fluoroscopy,
    updateInsertedLength,
    updateCatheterLength,
    updateDose,
    updateXrayTechnique,
    updateGuidewireResistance,
    updateGuidewireDiagnostics,
    setInjectButtonDisabled,
    setStopInjectionDisabled,
    updatePerfStats,
  };
}
