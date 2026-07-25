import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { AutomaticWithdrawalController } from '../src/ui/automaticWithdrawalController.js';

const controller = new AutomaticWithdrawalController();
assert.equal(controller.disabled, true, 'the automatic withdrawal button should be disabled at 0 cm');
assert.equal(controller.toggle(), false, 'automatic withdrawal should not start for an empty tool');

controller.updateLength(12.4);
assert.equal(controller.disabled, false);
assert.equal(controller.toggle(), true);
assert.equal(controller.command, -1, 'one click should continuously request withdrawal');

controller.updateLength(4.2);
assert.equal(controller.command, -1, 'automatic withdrawal should remain active while the tool is inserted');
controller.updateLength(0.04);
assert.equal(controller.command, 0, 'automatic withdrawal should stop at the sheath entrance');
assert.equal(controller.active, false);
assert.equal(controller.disabled, true);

controller.updateLength(3);
controller.toggle();
controller.cancel();
assert.equal(controller.command, 0, 'manual input should be able to cancel automatic withdrawal');

const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
assert.match(html, /id="guidewireAutoWithdraw"[\s\S]*?>Wysuń<\/button>/,
    'the guidewire row should expose its automatic withdrawal button');
assert.match(html, /id="catheterAutoWithdraw"[\s\S]*?>Wysuń<\/button>/,
    'the catheter row should expose its automatic withdrawal button');
