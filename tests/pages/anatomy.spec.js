import { test, expect } from '@playwright/test';

test('production page loads both anatomy models and the bundled skeleton worker', async ({ page }) => {
    const failures = [];
    const responses = [];
    page.on('pageerror', error => failures.push(error.message));
    page.on('console', message => {
        if (message.type() === 'error' || /Failed to load .*model/.test(message.text())) {
            failures.push(message.text());
        }
    });
    page.on('requestfailed', request => failures.push(`${request.url()}: ${request.failure()?.errorText}`));
    page.on('response', response => {
        responses.push({ url: response.url(), status: response.status() });
        if (response.status() >= 400) failures.push(`${response.status()} ${response.url()}`);
    });
    await page.goto('./');
    await expect(page.locator('body')).toHaveAttribute('data-skeleton-load', 'ready', { timeout: 120_000 });
    await expect(page.locator('body')).toHaveAttribute('data-aorta-load', 'ready', { timeout: 120_000 });
    expect(Number(await page.locator('body').getAttribute('data-skeleton-meshes'))).toBeGreaterThan(0);
    await expect(page.locator('#loadingScreen')).toHaveCount(0, { timeout: 15_000 });
    expect(responses.some(({ url }) => /\/assets\/skeleton\.worker-[^/]+\.js$/.test(url))).toBe(true);
    for (const suffix of ['.obj', '.stl', '.bin']) {
        expect(responses.some(({ url, status }) => url.endsWith(suffix) && status === 200)).toBe(true);
    }
    expect(responses.every(({ url }) => url.startsWith('http://127.0.0.1:4174/OpenEndovascularTrainer/'))).toBe(true);
    expect(failures).toEqual([]);
    await page.screenshot({ path: test.info().outputPath('loaded-anatomy.png') });
});
