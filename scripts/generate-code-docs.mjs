import { readFile, readdir, writeFile } from 'node:fs/promises';
import { extname, relative, resolve } from 'node:path';

const root = resolve(import.meta.dirname, '..');
const outputPath = resolve(root, 'docs/index.html');
const startMarker = '        <!-- GENERATED:MODULES:START -->';
const endMarker = '        <!-- GENERATED:MODULES:END -->';
const checkOnly = process.argv.includes('--check');

const escapeHtml = (value) => value
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;');

async function collectFiles(directory) {
    const entries = await readdir(directory, { withFileTypes: true });
    const files = await Promise.all(entries.map(async (entry) => {
        const path = resolve(directory, entry.name);
        return entry.isDirectory() ? collectFiles(path) : [path];
    }));
    return files.flat().filter((path) => ['.js', '.mjs'].includes(extname(path))).sort();
}

function declarations(source) {
    const results = [];
    const patterns = [
        /^(?:export\s+)?(?:async\s+)?function\s+([A-Za-z_$][\w$]*)\s*\(/gm,
        /^(?:export\s+)?class\s+([A-Za-z_$][\w$]*)\b/gm
    ];
    for (const pattern of patterns) {
        for (const match of source.matchAll(pattern)) results.push(match[1]);
    }
    return [...new Set(results)].sort();
}

function exportedNames(source) {
    const names = new Set();
    const direct = /^export\s+(?:default\s+)?(?:async\s+)?(?:function|class|const|let|var)\s+([A-Za-z_$][\w$]*)/gm;
    for (const match of source.matchAll(direct)) names.add(match[1]);
    for (const block of source.matchAll(/export\s*\{([^}]+)\}/gms)) {
        for (const item of block[1].split(',')) {
            const name = item.trim().split(/\s+as\s+/)[1] || item.trim().split(/\s+as\s+/)[0];
            if (name) names.add(name);
        }
    }
    return names;
}

function renderModule(path, source) {
    const file = relative(root, path).replaceAll('\\', '/');
    const all = declarations(source);
    const exported = exportedNames(source);
    const badges = all.map((name) => `<code${exported.has(name) ? ' class="exported"' : ''}>${escapeHtml(name)}${exported.has(name) ? ' ↗' : ''}</code>`).join(' ');
    return `          <tr><th scope="row"><a href="../${escapeHtml(file)}"><code>${escapeHtml(file)}</code></a></th><td>${badges || '<span class="muted">brak deklaracji nazwanych</span>'}</td></tr>`;
}

const files = await collectFiles(resolve(root, 'src'));
const rows = await Promise.all(files.map(async (path) => renderModule(path, await readFile(path, 'utf8'))));
const generated = `${startMarker}\n        <p class="generated-note">Wygenerowano automatycznie z <code>src/</code>: ${files.length} modułów. Symbol <strong>↗</strong> oznacza publiczny eksport.</p>\n        <div class="table-scroll"><table class="api-table"><thead><tr><th>Moduł</th><th>Nazwane funkcje i klasy</th></tr></thead><tbody>\n${rows.join('\n')}\n        </tbody></table></div>\n${endMarker}`;
const current = await readFile(outputPath, 'utf8');
const pattern = new RegExp(`${startMarker.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}[\\s\\S]*?${endMarker.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}`);
const updated = current.replace(pattern, generated);

if (updated === current) {
    console.log('Dokumentacja API jest aktualna.');
} else if (checkOnly) {
    console.error('Dokumentacja API jest nieaktualna. Uruchom: npm run docs:generate');
    process.exitCode = 1;
} else {
    await writeFile(outputPath, updated);
    console.log(`Zaktualizowano ${relative(root, outputPath)} (${files.length} modułów).`);
}
