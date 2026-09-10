import {pathToFileURL} from 'node:url';
const snapshot=process.env.OET_NODAL_AUDIT_ROOT ?? '/tmp/oet-production-nodal-wall-audit';
await import(pathToFileURL(`${snapshot}/probe.mjs`));
