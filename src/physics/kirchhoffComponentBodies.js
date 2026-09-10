/** Explicit mechanical component, with legacy pair aliases at the boundary.
 * No surrogate rod is introduced for an isolated tool.
 */
export function kirchhoffComponentBodies(component) {
    const bodies=component.bodies ?? [component.innerBody,component.outerBody];
    if(!Array.isArray(bodies)||bodies.length<1||bodies.length>2||bodies.some(body=>!body)||new Set(bodies).size!==bodies.length)
        throw new TypeError('One or two distinct Kirchhoff bodies are required');
    if(component.bodies && ((component.innerBody && component.innerBody!==bodies[0]) ||
        (component.outerBody && component.outerBody!==bodies[1])))throw new TypeError('Component body aliases disagree');
    return bodies.slice();
}
