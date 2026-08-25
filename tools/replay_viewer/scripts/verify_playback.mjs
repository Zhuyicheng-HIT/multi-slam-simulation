import { chromium } from 'playwright';
import { mkdir } from 'node:fs/promises';

const executablePath = '/home/ld666/.cache/ms-playwright/chromium-1234/chrome-linux64/chrome';
const output = new URL('../screenshots/', import.meta.url).pathname;
await mkdir(output, { recursive: true });
const browser = await chromium.launch({
  executablePath,
  headless: true,
  args: ['--no-sandbox', '--enable-webgl', '--use-angle=swiftshader-webgl'],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
const errors = [];
page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
page.on('pageerror', error => errors.push(error.message));
await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle' });
await page.waitForFunction(() => document.querySelectorAll('canvas').length === 2);
await page.waitForFunction(() => [...document.querySelectorAll('video')].every(video => video.readyState >= 2));
const playButton = page.locator('.play-button');
if (await playButton.getAttribute('aria-label') === '暂停') await playButton.click();
const timeline = page.locator('.timeline input');
await timeline.fill('23.2');
await page.waitForTimeout(1200);
await page.screenshot({ path: `${output}desktop-midpoint.png`, fullPage: true });
await page.locator('.local-panel canvas').screenshot({ path: `${output}canvas-local.png` });
await page.locator('.global-view canvas').screenshot({ path: `${output}canvas-global-midpoint.png` });
const midpoint = await page.locator('.pose-readout strong').nth(1).textContent();
await timeline.fill('0');
await page.waitForTimeout(300);
const start = await page.locator('.pose-readout strong').nth(1).textContent();
await timeline.fill('46.3');
await page.waitForTimeout(600);
const end = await page.locator('.pose-readout strong').nth(1).textContent();
await page.setViewportSize({ width: 390, height: 844 });
await timeline.fill('12');
await page.waitForTimeout(500);
await page.screenshot({ path: `${output}mobile.png`, fullPage: true });
console.log(JSON.stringify({ start, midpoint, end, canvases: await page.locator('canvas').count(), errors }, null, 2));
await browser.close();
