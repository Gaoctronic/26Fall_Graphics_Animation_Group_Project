const fs = require('fs');
const path = require('path');
const http = require('http');
const assert = require('assert/strict');

const playwrightPath = process.argv[2] || 'playwright';
const { chromium } = require(playwrightPath);
const root = path.resolve(__dirname, '../..');

(async () => {
  const errors = [];
  const server = http.createServer((req, res) => {
    const file = path.resolve(root, '.' + decodeURIComponent(req.url.split('?')[0]));
    if (!file.startsWith(root + path.sep)) return res.writeHead(403).end();
    fs.readFile(file, (error, data) => {
      if (error) return res.writeHead(404).end();
      const mime = {
        '.js': 'text/javascript; charset=utf-8',
        '.html': 'text/html; charset=utf-8',
        '.css': 'text/css; charset=utf-8',
        '.json': 'application/json; charset=utf-8',
        '.glb': 'model/gltf-binary',
      }[path.extname(file)] || 'application/octet-stream';
      res.setHeader('Content-Type', mime);
      res.end(data);
    });
  });

  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  const port = server.address().port;
  let browser;
  try {
    browser = await chromium.launch({
      executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe',
      headless: true,
      args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
    });
    const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
    page.on('pageerror', (error) => errors.push(`pageerror: ${error.message}`));
    page.on('console', (msg) => {
      if (msg.type() === 'error') errors.push(`console.error: ${msg.text()}`);
    });
    page.on('response', (response) => {
      if (response.status() >= 400) errors.push(`HTTP ${response.status()} ${response.url()}`);
    });

    await page.goto(`http://127.0.0.1:${port}/index.html`);
    await page.waitForFunction(() => (
      window.robotSimulation?.gateway
      && window.robotSimulation?.program?.runner
      && window.robotSimulation?.motion
      && window.robotSimulation?.cspace
      && document.querySelector('#tcp-pose')?.textContent?.includes('世界')
    ), { timeout: 20000 });

    console.log('--- [Browser 1] 未使能提交目标关节角任务 → 应转 ERROR 并返回明确原因 ---');
    const disabledCase = await page.evaluate(async () => {
      const api = window.robotSimulation;
      const res = await api.gateway.submitTask({
        task: 'avoid_move',
        goalQ: [0.35, 0.2, -0.15, 0.1, 0.25],
        algorithm: 'RRT*',
      }, { seed: 42, maxIterations: 300, stopOnFirstSolution: true });
      return { res, state: api.gateway.getState() };
    });
    assert.equal(disabledCase.res.ok, false);
    assert.equal(disabledCase.state.status, 'ERROR');
    assert.match(disabledCase.state.lastError, /未使能|急停/);
    console.log(`PASS [1]: status=${disabledCase.state.status} | reason=${disabledCase.state.lastError}`);

    console.log('--- [Browser 2] 使能后提交目标关节角任务 → 真实渲染帧运动到位后才显示 DONE ---');
    await page.check('#motion-enable');
    await page.waitForFunction(() => !document.querySelector('#angle-J1').disabled);
    await page.fill('#speed-override', '100');
    await page.locator('#speed-override').dispatchEvent('input');

    const goalQ = [0.45, 0.25, -0.2, 0.15, 0.35];
    // 启动任务并在真实 rAF 循环中观察中间 EXECUTING 状态与最终 DONE 到位误差
    const moveCase = await page.evaluate(async (target) => {
      const api = window.robotSimulation;
      api.gateway.reset();
      const statusTrace = [];
      let executingSnapshot = null;

      const unsub = api.gateway.subscribe((ev) => {
        if (statusTrace.at(-1) !== ev.status) statusTrace.push(ev.status);
        if (ev.status === 'EXECUTING' && !executingSnapshot) {
          const cur = api.joints.map((j) => j.angle);
          const maxErrAtStart = Math.max(...target.map((v, i) => Math.abs(v - cur[i])));
          executingSnapshot = {
            status: api.gateway.getState().status,
            runnerRunning: api.program.runner.state.running,
            maxErrAtStart,
          };
        }
      });

      const res = await api.gateway.submitTask({
        task: 'avoid_move',
        goalQ: target,
        algorithm: 'RRT*',
        speedPercent: 100,
      }, { seed: 42, maxIterations: 400, stopOnFirstSolution: true, execTimeoutMs: 30000 });
      unsub();

      const finalAngles = api.joints.map((j) => j.angle);
      const maxErrAtDone = Math.max(...target.map((v, i) => Math.abs(v - finalAngles[i])));
      return {
        ok: res.ok,
        status: api.gateway.getState().status,
        statusTrace,
        executingSnapshot,
        finalAngles,
        maxErrAtDone,
        dslScript: res.execResult?.dslScript,
        executedInstructions: res.execResult?.executedInstructions,
      };
    }, goalQ);

    assert.equal(moveCase.ok, true, '目标关节角任务应执行成功');
    assert.equal(moveCase.status, 'DONE');
    assert.deepEqual(moveCase.statusTrace, ['PLANNING', 'EXECUTING', 'DONE']);
    assert.ok(moveCase.executingSnapshot, '必须经过 EXECUTING 状态');
    assert.ok(moveCase.executingSnapshot.maxErrAtStart > 0.1, `进入 EXECUTING 时机械臂应尚未到位（实际初误差 ${moveCase.executingSnapshot.maxErrAtStart}）`);
    assert.ok(moveCase.maxErrAtDone < 0.01, `DONE 时机械臂必须已真实到位（实际误差 ${moveCase.maxErrAtDone} rad）`);
    console.log(
      `PASS [2]: trace=${moveCase.statusTrace.join('→')} | `
      + `EXECUTING初误差=${moveCase.executingSnapshot.maxErrAtStart.toFixed(4)} rad → `
      + `DONE末误差=${moveCase.maxErrAtDone.toFixed(6)} rad | `
      + `指令数=${moveCase.executedInstructions}`,
    );

    console.log('--- [Browser 3] 执行中途点击急停 #estop → 立即转 ERROR 且带急停原因 ---');
    // 启动反向长距离运动，然后点击界面急停按钮
    const estopStart = page.evaluate(() => {
      const api = window.robotSimulation;
      api.gateway.reset();
      window.__gwEstopPromise = api.gateway.submitTask({
        task: 'avoid_move',
        goalQ: [-0.55, -0.3, 0.25, -0.2, -0.4],
        algorithm: 'RRT*',
        speedPercent: 50,
      }, { seed: 42, maxIterations: 400, stopOnFirstSolution: true, execTimeoutMs: 20000 });
    });
    await estopStart;
    await page.waitForFunction(() => (
      window.robotSimulation.gateway.getState().status === 'EXECUTING'
      && window.robotSimulation.program.runner.state.running
    ), { timeout: 10000 });
    await page.waitForTimeout(150);
    await page.click('#estop');

    const estopCase = await page.evaluate(async () => {
      const res = await window.__gwEstopPromise;
      const api = window.robotSimulation;
      const stoppedAngles = api.joints.map((j) => j.angle);
      return { res, state: api.gateway.getState(), stoppedAngles };
    });
    assert.equal(estopCase.res.ok, false);
    assert.equal(estopCase.state.status, 'ERROR');
    assert.match(estopCase.state.lastError, /急停|未使能/);
    console.log(`PASS [3]: status=${estopCase.state.status} | reason=${estopCase.state.lastError}`);

    console.log('--- [Browser 4] 规划阶段目标碰撞障碍物 → 直接转 ERROR 且带碰撞原因 ---');
    await page.click('#release-estop');
    await page.check('#motion-enable');
    const collideCase = await page.evaluate(async () => {
      const api = window.robotSimulation;
      api.gateway.reset();
      const target = [0.2, 0.2, 0.0, 0.0, 0.0];
      api.cspace.setObstacles([
        { id: 'test-cspace-blocker', type: 'cspace-sphere', center: target, radius: 0.15 },
      ]);
      const res = await api.gateway.submitTask({
        task: 'avoid_move',
        goalQ: target,
        algorithm: 'RRT*',
      });
      api.cspace.setObstacles([]);
      return { res, state: api.gateway.getState() };
    });
    assert.equal(collideCase.res.ok, false);
    assert.equal(collideCase.state.status, 'ERROR');
    assert.match(collideCase.state.lastError, /碰撞/);
    console.log(`PASS [4]: status=${collideCase.state.status} | reason=${collideCase.state.lastError}`);

    assert.deepEqual(errors, [], `浏览器控制台或网络出现报错：${errors.join(' | ')}`);
    console.log('ALL GATEWAY BROWSER E2E CHECKS PASSED (0 console/page/HTTP errors).');
  } finally {
    if (browser) await browser.close();
    server.close();
  }
})();
