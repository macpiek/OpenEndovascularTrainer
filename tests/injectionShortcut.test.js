import assert from 'node:assert/strict';
import { shouldStartInjectionFromKeydown } from '../src/ui/injectionShortcut.js';

assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyI', repeat: false }, true),
    true,
    'the initial I keydown should start one injection'
);
assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyC', repeat: false }, true),
    true,
    'the initial C keydown should start one injection'
);
assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyI', repeat: true }, true),
    false,
    'holding I must not restart injection after the dose finishes'
);
assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyC', repeat: true }, true),
    false,
    'holding C must not restart injection after the dose finishes'
);
assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyI', repeat: false }, false),
    false,
    'the shortcut should remain disabled outside fluoroscopy'
);
assert.equal(
    shouldStartInjectionFromKeydown({ code: 'KeyR', repeat: false }, true),
    false,
    'unrelated shortcuts must not start injection'
);

console.log('injection shortcut tests passed');
