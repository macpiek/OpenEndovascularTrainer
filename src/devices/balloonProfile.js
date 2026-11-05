// Reliant product dimensions / free-inflation guide. The axial distribution
// below is a reduced constitutive preset, not a manufacturer measured shape.
export const RELIANT=Object.freeze({type:'reliant-balloon',shaftRadius:8/6,
    length:1000,minSheathFrench:12,maxVolumeMl:60,maxDiameterMm:46,
    distalOffset:8,sectionLength:72,fillRateMlS:6});
export const RELIANT_INFLATION_TABLE=Object.freeze([[0,8/3],[3,10],[9,20],[19,30],[41,40],[60,46]].map(Object.freeze));
const center=RELIANT.distalOffset+RELIANT.sectionLength/2;
// Nested positive annular-area layers give the tabulated diameter AND volume.
// Unlike an ellipsoid with a moving length, increasing fill cannot retract the
// shoulders or reduce capacity in an already wall-constrained cross-section.
const layers=RELIANT_INFLATION_TABLE.slice(1).map(([volume,diameter],i)=>{
    const [previousVolume,previousDiameter]=RELIANT_INFLATION_TABLE[i];
    const areaRadiusSquared=(diameter**2-previousDiameter**2)/4;
    const length=(volume-previousVolume)*1000/(Math.PI*areaRadiusSquared*2/3);
    return {previousVolume,volume,areaRadiusSquared,length};
});
export function validateBalloonInflation(volume) {
    if(!Number.isFinite(volume)||volume<0||volume>60)throw new RangeError('Balloon inflation must be within 0–60 ml');
}
export function balloonFreeRadiusAt(distanceFromTip,volume) {
    let r2=RELIANT.shaftRadius**2;
    for(const layer of layers) {
        const fraction=Math.max(0,Math.min(1,(volume-layer.previousVolume)/(layer.volume-layer.previousVolume)));
        const z=2*(distanceFromTip-center)/layer.length;
        r2+=fraction*layer.areaRadiusSquared*Math.max(0,1-z*z);
    }
    return Math.sqrt(r2);
}
export function balloonFreeDiameter(volume) {
    if(!Number.isFinite(volume))throw new RangeError('Finite balloon volume required');
    return 2*balloonFreeRadiusAt(center,Math.max(0,Math.min(60,volume)));
}
export const BALLOON_PROFILE_KNOTS=Object.freeze([center,...layers.flatMap(l=>[center-l.length/2,center+l.length/2])].sort((a,b)=>a-b));
