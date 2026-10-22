/** Two conservative compartments per renal segment: transit then parenchyma.
 * Inflow is the actual transported iodine leaving that segment's small arteries.
 * Timing/volumes are illustrative training presets, not patient calibration.
 */
export class RenalPerfusion {
    constructor(kidneys) {
        this.kidneys=kidneys.map(k=>({...k,transit:new Float64Array(5),tissue:new Float64Array(5)}));
        this.outlets=new Map();
        for(const kidney of this.kidneys) for(const terminal of kidney.terminalNodes)
            this.outlets.set(terminal.id,{kidney,sector:terminal.sector});
    }
    receive(nodeId,mass) {
        const target=this.outlets.get(nodeId);
        if(!target) return false;
        target.kidney.transit[target.sector]+=mass;
        return true;
    }
    update(dt) {
        if(!(dt>0))return 0;
        // Brief parenchymal phase, followed by rapid venous washout.
        const a=1/0.45,b=1/0.8,ea=Math.exp(-a*dt),eb=Math.exp(-b*dt);
        let drained=0;
        for(const k of this.kidneys) for(let i=0;i<5;i++) {
            const before=k.transit[i]+k.tissue[i];
            k.tissue[i]=k.tissue[i]*eb+k.transit[i]*a/(a-b)*(eb-ea);
            k.transit[i]*=ea;
            drained+=Math.max(0,before-k.transit[i]-k.tissue[i]);
        }
        return drained;
    }
    get massMg() {
        return this.kidneys.reduce((total,k)=>total+k.transit.reduce((a,b)=>a+b,0)+k.tissue.reduce((a,b)=>a+b,0),0);
    }
}
