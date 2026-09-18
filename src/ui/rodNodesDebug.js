import * as THREE from 'three';

/** Draw the accepted mechanical grid, never interpolated render-tube samples.
 * The catheter uses larger orange dots and the wire smaller cyan dots so both
 * remain visible at shared-axis nodes. No per-frame geometry allocation. */
export function createRodNodesDebug() {
    const group = new THREE.Group();
    const entries = [
        {color:0xffad42, size:9}, {color:0x31e5ff, size:4}
    ].map(({color,size}) => {
        const geometry = new THREE.BufferGeometry();
        const material = new THREE.PointsMaterial({color,size,sizeAttenuation:false,depthTest:false,depthWrite:false});
        const points = new THREE.Points(geometry, material);
        points.frustumCulled = false; points.renderOrder = 1001;
        group.add(points);
        return {geometry,material,points,view:null,capacity:0};
    });
    return {group,
        update(wire,catheter,visible) {
            group.visible = visible;
            if (!visible) return;
            [catheter,wire].forEach((body,index) => {
                const entry = entries[index], view = body?.jointStateView;
                if (view && entry.view === view) return;
                entry.view = view;
                const start = body?.activeStart ?? 0;
                const count = view ? view.positions.length / 3 : body ? Math.max(0,body.activeEnd-start+1) : 0;
                if (count > entry.capacity) {
                    entry.capacity = Math.max(count, entry.capacity * 2, 64);
                    // Release the previous GPU buffer before changing capacity.
                    entry.geometry.dispose();
                    entry.geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(entry.capacity*3),3).setUsage(THREE.DynamicDrawUsage));
                }
                const attribute = entry.geometry.getAttribute('position');
                if (attribute) {
                    if (view) attribute.array.set(view.positions);
                    else for(let i=0;i<count;i++)attribute.array.set([body.x[start+i],body.y[start+i],body.z[start+i]],3*i);
                    attribute.needsUpdate = true;
                }
                entry.geometry.setDrawRange(0,count);
            });
        },
        dispose() {for(const entry of entries){entry.geometry.dispose();entry.material.dispose();}group.removeFromParent();}
    };
}
