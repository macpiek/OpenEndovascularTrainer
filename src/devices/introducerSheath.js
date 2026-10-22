import {INTRODUCER_SHEATH_WALL_THICKNESS_MM} from '../toolDimensions.js';
import {sampleSheathPath, sheathLength} from '../physics/sheathPath.js';

export const INTRODUCER_LENGTHS_MM = Object.freeze([70, 450, 900]);
export const INTRODUCER_SIZES_F = Object.freeze([6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 22, 24]);
export const GUIDEWIRE_LENGTHS_MM = Object.freeze([1000, 1500, 1800, 2000]);

export function introducerDimensions(french) {
    if (!INTRODUCER_SIZES_F.includes(french)) throw new RangeError('Nieznana średnica koszulki');
    // French labels the lumen capacity; the outside includes both walls.
    return {french, radius:french / 6 + INTRODUCER_SHEATH_WALL_THICKNESS_MM, innerRadius:french / 6};
}

export function canSelectIntroducer(sheath) {
    return sheathLength(sheath) === 0;
}

/** Select a replacement outside the vessel; insertion remains a manual action. */
export function prepareIntroducerSelection(options) {
    if (!canSelectIntroducer(options.sheath))
        throw new Error('Całkowicie wycofaj koszulkę, aby zmienić jej długość lub średnicę.');
    return prepareIntroducerExchange({...options, insertion:0});
}

// Extend the free tip by a fixed arc-length step. Wall projection bends the
// direction instead of shortening the step or advancing through the wall.
function advanceFreeTip(previous, direction, step, radius, contactAt) {
    if (!contactAt) throw new Error('Brak kontaktu ze ścianą dla swobodnej końcówki koszulki.');
    // Spread an approaching bend over the distal section instead of waiting
    // for a head-on tip collision and creating a single sharp hinge.
    const lookAhead=8, probe=contactAt({x:previous.x+direction.tx*lookAhead,
        y:previous.y+direction.ty*lookAhead,z:previous.z+direction.tz*lookAhead});
    if (probe.signedDistance < radius+.5 && probe.normal) {
        const n=probe.normal,dot=n.x*direction.tx+n.y*direction.ty+n.z*direction.tz;
        const x=n.x-dot*direction.tx,y=n.y-dot*direction.ty,z=n.z-dot*direction.tz,d=Math.hypot(x,y,z);
        if(d>1e-6) {
            const bend=Math.min(.15*step/Math.max(1,radius),(radius+.5-probe.signedDistance)/lookAhead);
            const tx=direction.tx+x/d*bend,ty=direction.ty+y/d*bend,tz=direction.tz+z/d*bend;
            const norm=Math.hypot(tx,ty,tz);direction={tx:tx/norm,ty:ty/norm,tz:tz/norm};
        }
    }
    let p = {x:previous.x+direction.tx*step, y:previous.y+direction.ty*step, z:previous.z+direction.tz*step};
    for (let iteration = 0; iteration < 48; iteration++) {
        const contact = contactAt(p), gap = radius + .06 - contact.signedDistance;
        if (!Number.isFinite(gap)) break;
        if (gap <= 1e-5) {
            const turn = ((p.x-previous.x)*direction.tx + (p.y-previous.y)*direction.ty +
                (p.z-previous.z)*direction.tz) / step;
            if (turn < Math.cos(.35)) break;
            return p;
        }
        const n = contact.normal;
        if (!n || ![n.x,n.y,n.z].every(Number.isFinite)) break;
        p.x += n.x*gap; p.y += n.y*gap; p.z += n.z*gap;
        const dx=p.x-previous.x,dy=p.y-previous.y,dz=p.z-previous.z;
        const chord=Math.hypot(dx,dy,dz);
        if (chord < 1e-8) break;
        p={x:previous.x+dx*step/chord,y:previous.y+dy*step/chord,z:previous.z+dz*step/chord};
    }
    throw new Error('Końcówka koszulki opiera się o ścianę naczynia. Wycofaj ją lub zmień przebieg prowadnikiem.');
}

/** Follow deployed support, then bend the free tip against the vessel wall. */
export function prepareIntroducerExchange({sheath, length, french, transport, catheterProgress = 0, catheterRadius = 5/6, sampleWire, sampleCatheter, clearanceAt, contactAt, insertion = Math.min(sheathLength(sheath), length)}) {
    if (!INTRODUCER_LENGTHS_MM.includes(length)) throw new RangeError('Nieznana długość koszulki');
    const dimensions = introducerDimensions(french);
    if (!Number.isFinite(insertion) || insertion<0 || insertion>length) throw new RangeError('Nieprawidłowe wsunięcie koszulki');
    if (catheterProgress>0 && catheterRadius>dimensions.innerRadius+1e-9) throw new Error('Cewnik nie mieści się w wybranej koszulce. Wybierz większą średnicę.');
    const wireProgress = transport?.progress ?? 0;
    const base = sheath.shortPath ?? [{...sheath.start,s:0}, {...sheath.end,s:70}];
    const shortSheath = {start:base[0],end:base.at(-1),path:base};
    // Negative support keeps the fixed access origin and loading direction even
    // when the introducer is completely withdrawn (insertion = 0).
    const path = [{...sampleSheathPath(shortSheath,-2),s:-2}];
    const coordinates=[];
    for(let s=0;s<insertion-1e-7;s+=.5)coordinates.push(s);
    coordinates.push(insertion);
    for (const s of coordinates) {
        let freeTip = false;
        let p = s <= 70 ? sampleSheathPath(shortSheath, s) :
            sheath.path && s<=sheathLength(sheath) ? sampleSheathPath(sheath,s) :
            s<=catheterProgress && sampleCatheter ? sampleCatheter(s) :
            s<=wireProgress && sampleWire ? sampleWire(s) : null;
        const retained=sheath.path&&s<=sheathLength(sheath)&&sheath.french===french;
        if (s>70 && !p) {
            const previous=path.at(-1), before=path.at(-2);
            const d=Math.hypot(previous.x-before.x,previous.y-before.y,previous.z-before.z);
            const direction={tx:(previous.x-before.x)/d,ty:(previous.y-before.y)/d,tz:(previous.z-before.z)/d};
            p=advanceFreeTip(previous,direction,s-previous.s,dimensions.radius,contactAt);
            freeTip=true;
        }
        if (!p || ![p.x,p.y,p.z].every(Number.isFinite)) throw new Error('Brak ciągłego przebiegu prowadnika.');
        // A wire may hug the wall. The wider introducer displaces that route
        // slightly inward; reject narrow lumens rather than tunnelling through.
        if (s > 70 && contactAt && !freeTip && !retained) {
            const original = p;
            p = {...p};
            for (let attempt=0; attempt<16; attempt++) {
                const contact=contactAt(p), gap=dimensions.radius+0.06-contact.signedDistance;
                if (gap<=0) break;
                const n=contact.normal;
                if (!n || !Number.isFinite(gap)) break;
                const shift=Math.min(gap,0.5);
                p.x+=n.x*shift; p.y+=n.y*shift; p.z+=n.z*shift;
                if (Math.hypot(p.x-original.x,p.y-original.y,p.z-original.z)>2*dimensions.radius) break;
            }
        }
        if (s > 70 && !retained && clearanceAt && clearanceAt(p) < dimensions.radius + 0.05)
            throw new Error('Koszulka nie mieści się na tym przebiegu prowadnika. Wybierz mniejszą średnicę lub popraw położenie prowadnika.');
        if (s > 70 && freeTip && contactAt) {
            const previous=path.at(-1);
            const midpoint={x:(previous.x+p.x)/2,y:(previous.y+p.y)/2,z:(previous.z+p.z)/2};
            if (contactAt(midpoint).signedDistance < dimensions.radius + .02)
                throw new Error('Koszulka opiera się o ścianę na zakręcie naczynia.');
        }
        path.push({x:p.x,y:p.y,z:p.z,s});
    }
    return {...sheath,...dimensions,length,insertion,entryLength:base.at(-1).s,path,shortPath:base,start:{...sampleSheathPath(shortSheath,0)},end:{...path.at(-1)}};
}

/** Independent manual feed; the selected nominal length does not change. */
export function prepareIntroducerMotion(options, delta) {
    const length=Math.round(options.sheath.length??70);
    const insertion=Math.max(0,Math.min(length,sheathLength(options.sheath)+delta));
    if(insertion===sheathLength(options.sheath))return null;
    return prepareIntroducerExchange({...options,length,french:options.sheath.french??6,insertion});
}
