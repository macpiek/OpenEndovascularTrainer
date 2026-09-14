import { defineConfig } from 'vite';
import { writeFile } from 'node:fs/promises';

export default defineConfig({
    build: { rollupOptions: { input: ['index.html', 'shared-axis-lab.html'] } },
    plugins: [{name:'local-physics-capture',configureServer(server){
        server.middlewares.use('/__physics-capture',async(req,res)=>{
            if(req.method!=='POST'||!['127.0.0.1','::1','::ffff:127.0.0.1'].includes(req.socket.remoteAddress)||
                req.headers.origin!==`http://${req.headers.host}`){res.statusCode=403;res.end();return;}
            try{
                const chunks=[];let size=0;
                for await(const chunk of req){size+=chunk.length;if(size>16*1024*1024)throw new Error('Capture too large');chunks.push(chunk);}
                const payload=Buffer.concat(chunks).toString('utf8'),data=JSON.parse(payload);
                if(!Number.isInteger(data.count)||!Number.isInteger(data.band)||!Array.isArray(data.matrix)||
                    data.matrix.length!==data.count*data.band)throw new Error('Invalid frozen operator');
                await writeFile('/tmp/oet-frozen-coupled.json',payload+'\n');
                res.end('saved');
            }catch(error){res.statusCode=400;res.end(error.message);}
        });
    }}],
    optimizeDeps: {
        include: [
            'three',
            'three-mesh-bvh',
            'three/examples/jsm/loaders/OBJLoader.js',
            'three/examples/jsm/loaders/STLLoader.js'
        ]
    }
});
