import {initializeCompositeAppState,prepareCompositeAppInputs,completeCompositeAppNativePositionBoundaries} from './kirchhoffCompositeAppInputs.js';
import {createCompositeJointWorldAdapter} from './kirchhoffCompositeJointWorldAdapter.js';
import {createCompositeJointTimeStepState} from './kirchhoffCompositeJointTimeStep.js';
import {transferCompositeNativeState} from './kirchhoffCompositeNativeTransfer.js';
import {createCompositeAppLocalFrame} from './kirchhoffCompositeAppLocalFrame.js';

function stateSnapshot(state) {
    const own=createCompositeJointTimeStepState(state);own.materialVelocities=structuredClone(state.materialVelocities);
    own.boundaryMultipliers=structuredClone(state.boundaryMultipliers);return own;
}

function readonlyBindings(bindings) {
    return Object.freeze(bindings.map(binding=>Object.freeze({toolId:binding.toolId,body:binding.body,
        nodes:Object.freeze(binding.nodes.map(row=>Object.freeze({...row}))),
        edges:Object.freeze(binding.edges.map(row=>Object.freeze({...row})))})));
}

/** Actual app factory for the newest common/relative Joint solver. Source bodies
 * are imported once per reset; accepted Float64 state remains authoritative.
 * The World adapter owns publication, dt retries, source constraint guards and
 * sheath pressure. This factory cannot disable unsupported active contacts.
 *
 * readToolSources(world) -> explicit createCompositeAppToolSource descriptors.
 * readControls({world,state,bindings,mappings,dt}) -> prepareCompositeAppInputs
 * arguments (commands, positionBoundaries, reservoir, loads and budgets).
 * state/mappings passed to that callback are independent read snapshots.
 *
 * Native readNativeLayout({world,state,bindings,dt}) -> {tools,reservoir}
 * supplies each body's current node coordinates and own material labels.
 * The adapter privately remaps accepted material before the next Lagrangian
 * dt (labelShift:0, feedVelocity:0); the fixed valve boundary then transports
 * material. New proximal material needs explicit reservoir pose and rates.
 * Grid changes use ordinary native sampling, never the transported Float32
 * body views as replacement accepted geometry. Retries reuse the same remap.
 *
 * coordinateOrigin:'sheath-start' (or a fixed world XYZ) keeps accepted native
 * poses and reservoir/history permanently local. Callbacks receive the actual
 * global World and LOCAL state; positionBoundaries remain global UI inputs.
 * Only publication adds the origin back. Defaults retain global coordinates.
 */
export function createCompositeJointAppSystem({readToolSources,readControls,readNativeLayout,worldWall,coordinateOrigin,cooperative=false,workSliceMs=0,geometry='continuous-material-frame',
    id=geometry==='native-discrete-rod'?'composite-joint-native':'composite-joint-continuous'}={}) {
    if(typeof readToolSources!=='function'||typeof readControls!=='function')
        throw new TypeError('Application Joint system requires source and control callbacks');
    if(!['continuous-material-frame','native-discrete-rod'].includes(geometry))throw new RangeError('Unknown application Joint geometry');
    if(readNativeLayout!==undefined&&(typeof readNativeLayout!=='function'||geometry!=='native-discrete-rod'))
        throw new TypeError('Native layout preparation requires the native Joint model and a callable source');
    if(coordinateOrigin!==undefined&&geometry!=='native-discrete-rod')throw new RangeError('Local app coordinates require the native Joint geometry');
    let initial=null,frame=null,budget=null;
    const sourceWorld=world=>frame?.sourceWorld??world;
    const makeAdapter=()=>createCompositeJointWorldAdapter({id,elementBackend:'wasm-exact',worldWall:frame?frame.wallConfiguration(worldWall):worldWall,worldSheath:true,worldContainment:geometry==='native-discrete-rod',cooperative,workSliceMs,
        initialize(world) {
            const tools=readToolSources(sourceWorld(world));
            if(!Array.isArray(tools))throw new TypeError('Application source callback must return synchronous tool descriptors');
            const step=world.stepCount??0;
            const imported=initializeCompositeAppState({tools,time:world.time??step*world.fixedDt,step,geometry});
            initial={bindings:readonlyBindings(imported.bindings),mappings:structuredClone(imported.mappings)};
            return {state:frame?frame.initialState(imported.state):imported.state,bindings:initial.bindings,
                ...(frame?{publicationOrigin:frame.origin}:{})};
        },
        ...(readNativeLayout?{transferStep({world,state,bindings,dt}) {
            const input=readNativeLayout({world:sourceWorld(world),state:stateSnapshot(state),bindings:readonlyBindings(bindings),dt});
            if(!input||input.then)throw new TypeError('Native layout callback must return synchronous explicit inputs');
            const next=transferCompositeNativeState({contactRateMode:worldWall?.rateMode,...input,state,bindings});
            if(next)initial.mappings=structuredClone(next.mappings);
            return next;
        }}:{}),
        prepareStep({world,state,bindings,dt}) {
            const controls=readControls({world:sourceWorld(world),state:stateSnapshot(state),dt,bindings:readonlyBindings(bindings),
                mappings:structuredClone(initial.mappings)});
            if(!controls||controls.then||Object.getPrototypeOf(controls)!==Object.prototype)
                throw new TypeError('Application control callback must return synchronous explicit inputs');
            // Neither callback-provided state nor dt can replace the adapter's
            // accepted state and pending physical timestep.
            if(controls.state!==undefined||controls.dt!==undefined||controls.preparedState!==undefined)
                throw new RangeError('Application controls cannot replace the accepted state or dt');
            const {worldWallSurfacePosePaths,...inputs}=frame?frame.controls(controls):controls,
                prepared=prepareCompositeAppInputs({...inputs,positionBoundaries:completeCompositeAppNativePositionBoundaries({state,bindings,positionBoundaries:inputs.positionBoundaries}),state,dt});
            return {...prepared.options,preparedState:prepared.state,
                ...(worldWallSurfacePosePaths===undefined?{}:{worldWallSurfacePosePaths})};
        }
    });
    let adapter=coordinateOrigin===undefined?makeAdapter():null;
    return Object.freeze({id,
        step(world,dt) {
            if(coordinateOrigin!==undefined&&!frame) {
                frame=createCompositeAppLocalFrame(world,coordinateOrigin);adapter=makeAdapter();adapter.setBudget(budget);
            }
            if(frame&&frame.sourceWorld!==world)throw new RangeError('A local Joint system belongs to exactly one World');
            return adapter.step(frame?.world??world,dt);
        },
        reset(world){
            if(frame&&frame.sourceWorld!==world)throw new RangeError('A local Joint system belongs to exactly one World');
            adapter?.reset(frame?.world??world);initial=null;if(frame){frame=null;adapter=null;}
        },
        setBudget(value){budget=value;return adapter?.setBudget(value);},
        snapshot:()=>adapter?.snapshot()??null,
        get diagnostics(){return {...(adapter?.diagnostics??{initializations:0,preparations:0,attempts:0,accepted:0,rejected:0,publications:0,
            pending:false,stateTime:null,stateStep:null,nodeCount:null,lastStatus:null,workspace:null}),appSourceInitialized:initial!==null,
            coordinateFrame:coordinateOrigin===undefined?'world':'constant-local',coordinateOrigin:frame?.origin.slice()??null,
            elasticityGeometry:geometry,lengthGeometry:geometry==='continuous-material-frame'?'continuous-arclength':'native-chords',
            activeRangeTransferReady:readNativeLayout!==undefined};}
    });
}
