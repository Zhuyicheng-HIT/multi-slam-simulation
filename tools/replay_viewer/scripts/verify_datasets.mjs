import { chromium } from 'playwright';

const browser = await chromium.launch({
  executablePath: '/home/ld666/.cache/ms-playwright/chromium-1234/chrome-linux64/chrome',
  headless: true,
  args: ['--no-sandbox', '--enable-webgl', '--use-angle=swiftshader-webgl'],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
const errors = [];
page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
page.on('pageerror', error => errors.push(error.message));
await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle' });
await page.locator('.play-button').click();
await page.locator('.timeline input').fill('23.2');
await page.waitForTimeout(800);
const m2dgrPoints = await page.locator('.pose-readout strong').nth(1).textContent();
await page.locator('.dataset-select select').selectOption('r3live-degenerate-02');
await page.waitForFunction(() => document.querySelector('.duration')?.textContent === '01:41.88');
await page.locator('.play-button').click();
await page.locator('.timeline input').fill('50');
await page.waitForTimeout(1200);
await page.screenshot({ path: 'screenshots/r3live-midpoint.png', fullPage: true });
const result = {
  selected: await page.locator('.dataset-select select').inputValue(),
  duration: await page.locator('.duration').textContent(),
  m2dgrPoints,
  r3liveMapMessage: await page.locator('.map-unavailable strong').textContent(),
  r3liveLidarFrame: await page.locator('.local-panel .badge').textContent(),
  depthDisabled: await page.locator('.segmented button').nth(1).isDisabled(),
  videoReady: await page.locator('.camera-feed video').evaluate(video => video.readyState),
  errors,
};
console.log(JSON.stringify(result, null, 2));
await browser.close();
