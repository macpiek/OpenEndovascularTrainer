import {sewnTopology} from './stentGraftSewnTopology.js';

// The mechanical mouth samples belong to the exact same polygonal edges as
// the visible/collision Y mesh, with no axial overlap or independent fan.
export function junctionBindings(device) {
    return sewnTopology(device).bindings;
}
