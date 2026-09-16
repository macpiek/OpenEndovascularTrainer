import { build } from 'vite';
import { cp, readFile, writeFile } from 'node:fs/promises';

// Pages serves this repository below a project path, not the domain root.
// Keep the deploy artifact separate from the historical, tracked dist/ files.
const base = process.env.PAGES_BASE_PATH ?? '/OpenEndovascularTrainer/';
if (!base.startsWith('/') || base.includes('..') || base.includes('?') || base.includes('#')) {
    throw new Error('PAGES_BASE_PATH must be an absolute URL path');
}
await build({ base: `${base.replace(/\/+$/, '')}/`, build: { outDir: '.pages-dist' } });
await cp('docs', '.pages-dist/docs', { recursive: true });
// Documentation source links still work when only the build is published.
const docsPath = '.pages-dist/docs/index.html';
const docs = await readFile(docsPath, 'utf8');
const repository = process.env.GITHUB_REPOSITORY ?? 'macpiek/OpenEndovascularTrainer';
const revision = process.env.GITHUB_SHA ?? 'main';
await writeFile(docsPath, docs.replaceAll('href="../src/',
    `href="https://github.com/${repository}/blob/${revision}/src/`));
await writeFile('.pages-dist/.nojekyll', '');
