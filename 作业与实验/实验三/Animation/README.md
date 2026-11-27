# 实验三：避障工位、三段式抓取与接触力控

本目录是可独立启动的实验三工程，基于组长提供的实验三组员开工包，包含张云乔（组员 B）的三项代码交付。入口保持为根目录 `index.html`，JavaScript 位于 `js/`，样式位于 `css/`，实际使用的模型与配置位于 `images/`。

## 运行

用 VS Code 打开本目录，通过 Live Server 启动 `index.html`。也可在本目录执行：

```powershell
python -m http.server 8000 --bind 127.0.0.1
```

打开 `http://127.0.0.1:8000/index.html`。Three.js 固定为本地 0.186.1，运行页面和 Node 单元测试均无需安装 npm 依赖。

桌面上可看到两个红色障碍柱 `pillar_1`、`pillar_2`，绿色区域为 `zone_B`。鼠标左键旋转，滚轮缩放，右键平移。

## 组员 B 的变更

- `js/Workstation.js`：障碍柱、B 区、半透明安全膨胀边界、障碍规格与 B 区位姿接口、可保持的夹爪开度和接触力状态。
- `js/grasp-planner.js`：三段式 TCP 位姿生成与默认 50N 的接触阈值仿真。
- `js/tests/grasp-planner.test.mjs`：10 项测试，覆盖几何位姿、双侧接触与空夹、释放复位、边界显示及碰撞正反例。

其余应用代码、测试和资源来自开工包，作为运行基础保留；本次没有修改其他组员的业务模块。此 README 为本次提交补充的运行与交接说明。

## 接口与约定

```javascript
workstation.getObstacleSpecs(); // pillar_1、pillar_2、table；世界坐标 ObstacleSpec[]
workstation.setObstacleInflationVisible(true, 0.03);
workstation.getZoneBPose(); // { position: THREE.Vector3, quaternion: THREE.Quaternion }

planThreeStageGrasp(targetSpec, { approachDist: 0.08, liftHeight: 0.12 });
// -> { preGrasp, grasp, lift }；每一项为 { position, quaternion }

simulateEffortGrip(workstation, 50);
// -> { gripped, contactForceN, reason }
```

长度统一沿用现有模型的资产单位，尚未换算为米。`targetSpec.position` 是直立物体的世界中心，`size` 是尺寸；可选 `axis` 的 XZ 投影指定夹爪闭合方向。默认从上方接近，抓取 TCP 位于物体顶面中心，然后沿世界 +Y 抬升。三个输出是几何目标位姿，运动执行与 IK/避障验证由调用方负责。

`getZoneBPose()` 返回 B 区桌面中心位姿。放置工件中心需加半高，再按工具偏移换算 TCP。安全边界显示与碰撞余量使用同一 `margin`；隐藏边界不会关闭余量，修改余量后须重新把障碍规格传给碰撞器。

力控采用任务契约规定的教学简化模型：逐步闭爪，首次包围盒接触即认为达到给定阈值并停止；双指接触才挂载工件，单侧接触停止并报告失败。它不求解材料刚度或动力学。调用前应停止机械臂运动；`OPEN` 和复位操作会清除力控状态。

## 联调边界

当前页面可以运行并展示工位，但尚未接入实验三的整段自动抓取任务和力控按钮。现有页面 `GRIP CLOSE` 仍使用实验二的距离抓取逻辑。

- 组长在总装中把 `workstation.getObstacleSpecs()` 传给 C-space；执行层的额外障碍集合也需包含柱体。
- 组员 A 调用 `setObstacleInflationVisible()` 控制显示。
- 组员 C/组长调用抓取位姿、B 区位姿和力控函数完成任务编排；现有 DSL 不因新增此模块而自动支持 `EFFORT` 语法。

可在浏览器开发者控制台独立查看边界并把障碍规格注入当前规划碰撞器：

```javascript
const api = window.robotSimulation;
api.workstation.setObstacleInflationVisible(true, 0.03);
api.cspace.setObstacles(api.workstation.getObstacleSpecs());
const graspModule = await import('./js/grasp-planner.js');
```

## 测试

在本目录运行新增测试：

```powershell
node js/tests/grasp-planner.test.mjs
```

运行全部 14 份 Node 测试脚本：

```powershell
Get-ChildItem js/tests/*.test.mjs | ForEach-Object {
  node $_.FullName
  if ($LASTEXITCODE -ne 0) { throw "测试失败：$($_.Name)" }
}
```

原有 `group-b.test.mjs` 会生成 `组员B_坐标验证数据.json`；该文件是测试输出，不属于本次提交。保留的 `.cjs` 浏览器测试需另有本机 Playwright 环境，它不是页面运行依赖。

本次仅收录实际使用的五项模型资源/配置、本地库及许可证、应用与测试代码。旧验证数据、模型结构转储、未使用的模型、缓存、日志和 `node_modules/` 不纳入新增工程。
