import { defineConfig } from '@playwright/test';

export default defineConfig({
    testDir: './tests/pages',
    timeout: 180_000,
    workers: 1,
    reporter: [['list']],
    use: {
        baseURL: 'http://127.0.0.1:4174/OpenEndovascularTrainer/',
        headless: true,
        viewport: { width: 1280, height: 900 },
        screenshot: 'only-on-failure',
        trace: 'retain-on-failure',
        launchOptions: { args: ['--enable-unsafe-swiftshader'] }
    },
    webServer: {
        command: 'vite preview --outDir .pages-dist --base /OpenEndovascularTrainer/ --host 127.0.0.1 --port 4174 --strictPort',
        url: 'http://127.0.0.1:4174/OpenEndovascularTrainer/',
        reuseExistingServer: false
    }
});
