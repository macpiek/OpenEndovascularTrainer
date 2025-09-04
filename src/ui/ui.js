import { PatientMonitor } from './patientMonitor.js';
import { initCArmPreview, cArmPreviewGroup, cArmPreviewGantry } from './carmPreview.js';
import { setupCArmControls } from '../carm.js';
import { setBendingStiffness, setWallFriction, setSmoothingIterations } from '../physics/elasticRod.js';

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
    document.getElementById('bpValue')
  );

  // C-arm UI preview + controls
  initCArmPreview();
  setupCArmControls(camera, vessel, cameraRadius, cArmPreviewGroup, cArmPreviewGantry);

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
  const persistenceSlider = document.getElementById('persistence');
  const noiseSlider = document.getElementById('noiseLevel');
  const insertedLengthEl = document.getElementById('insertedLength');
  const doseDisplayEl = document.getElementById('currentDose');
  const perfStatsEl = document.getElementById('perfStats');

  // Initial UI state
  if (voxelRenderToggle) {
    voxelGroup.visible = voxelRenderToggle.checked;
  }

  // Avoid sticky focus on sliders
  const sliders = [
    bendSlider,
    staticFricSlider,
    kineticFricSlider,
    smoothIterSlider,
    persistenceSlider,
    noiseSlider,
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
  if (noiseSlider) {
    displayMaterial.uniforms.noiseLevel.value = parseFloat(noiseSlider.value);
    noiseSlider.addEventListener('input', e => {
      displayMaterial.uniforms.noiseLevel.value = parseFloat(e.target.value);
    });
  }
  if (persistenceSlider) {
    blendMaterial.uniforms.decay.value = parseFloat(persistenceSlider.value);
    persistenceSlider.addEventListener('input', e => {
      blendMaterial.uniforms.decay.value = parseFloat(e.target.value);
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
    modeToggle.textContent = 'Wireframe';
    displayMaterial.uniforms.fluoroscopy.value = true;
    modeToggle.addEventListener('click', () => {
      fluoroscopy = !fluoroscopy;
      displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
      modeToggle.textContent = fluoroscopy ? 'Wireframe' : 'Fluoroscopy';
      // Render the guidewire in white so it appears black after inversion
      if (wireMaterial) wireMaterial.color.set(0xffffff);
      if (typeof onModeChange === 'function') onModeChange(fluoroscopy);
    });
    // Set initial mode state to simulator
    if (typeof onModeChange === 'function') onModeChange(fluoroscopy);
  }

  // Keyboard controls for guidewire advance and keyboard-triggered injection
  let advance = 0;
  document.addEventListener('keydown', e => {
    if (e.code === 'KeyW' || e.code === 'ArrowUp') {
      advance = 1; e.preventDefault();
    }
    if (e.code === 'KeyS' || e.code === 'ArrowDown') {
      advance = -1; e.preventDefault();
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
  }, true);

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
    if (insertedLengthEl) insertedLengthEl.textContent = cm.toFixed(1) + ' cm';
  }
  function updateDose(ml) {
    if (doseDisplayEl) doseDisplayEl.textContent = ml.toFixed(1) + ' ml';
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
    getFluoroscopy: () => fluoroscopy,
    updateInsertedLength,
    updateDose,
    setInjectButtonDisabled,
    setStopInjectionDisabled,
    updatePerfStats,
  };
}

