// 轨迹三维视图：把剩余航点与每段方向向量画在世界坐标系里，只占档位行右侧一小块。
// three.js 从 CDN 动态 import：库取不到或没有 WebGL 时，只有这一块降级，控制台其余照常。

const CDN = "https://cdn.jsdelivr.net/npm/three@0.187.0/build/three.module.js";
// 取库有 8 秒上限，超时按不可用处理。
const CDN_TIMEOUT_MS = 8000;

const panel = document.getElementById("scene-card");
const canvas = document.getElementById("scene-canvas");
// 就绪后才显示，不留空白画布。
panel.hidden = true;

let THREE = null;
let renderer = null;
let scene = null;
let camera = null;
let path = null;
let origin = { x: 0, y: 0, z: 0 };
let view = { radius: 60, theta: 0, phi: 1.05 };

function fetchLibrary() {
  const timeout = new Promise((_, reject) =>
    setTimeout(() => reject(new Error("CDN 未在时限内响应")), CDN_TIMEOUT_MS)
  );
  return Promise.race([import(CDN), timeout]);
}

function unavailable(reason, error) {
  console.warn(`3D 视图不可用：${reason}`, error);
  panel.hidden = true;
}

// NED → 屏幕：东→右，下→反上，北→里。以载具为原点，数值才不会大到丢精度。
function toScene(point) {
  return new THREE.Vector3(
    point.y - origin.y,
    -(point.z - origin.z),
    -(point.x - origin.x)
  );
}

// 机头方向：NED 偏航 0° 朝北，映射到屏幕就是 -z。
function headingVector(yawDeg) {
  const rad = (Number(yawDeg) * Math.PI) / 180;
  return new THREE.Vector3(Math.sin(rad), 0, -Math.cos(rad));
}

function clearPath() {
  if (!path) return;
  for (const node of [...path.children]) {
    node.geometry?.dispose();
    node.material?.dispose();
    path.remove(node);
  }
}

function drawPoint(position, radius, color) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 12, 10),
    new THREE.MeshBasicMaterial({ color })
  );
  mesh.position.copy(position);
  path.add(mesh);
}

function drawLeg(from, to, color) {
  const length = to.distanceTo(from);
  if (length < 1e-3) return;
  const arrow = new THREE.ArrowHelper(
    to.clone().sub(from).normalize(),
    from,
    length,
    color,
    Math.min(1.6, length * 0.35),
    Math.min(0.7, length * 0.16)
  );
  path.add(arrow);
}

function draw(snapshot) {
  const vehicle = snapshot.vehicle || null;
  const points = Array.isArray(snapshot.points) ? snapshot.points : [];
  const flat = snapshot.vertical_motion === false;
  clearPath();
  if (!vehicle) return;
  origin = { x: vehicle.x, y: vehicle.y, z: vehicle.z };

  const home = toScene(vehicle);
  const stops = points.map((point) =>
    toScene(flat ? { ...point, z: vehicle.z } : point)
  );

  const span = Math.max(12, ...stops.map((node) => node.distanceTo(home)));

  const line = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([home, ...stops]),
    new THREE.LineBasicMaterial({ color: 0x4cc2a5 })
  );
  path.add(line);

  let previous = home;
  stops.forEach((node, index) => {
    drawLeg(previous, node, 0x9fd8cf);
    if (index === 0) drawPoint(node, 1.0, 0xffc14d);
    previous = node;
  });

  drawPoint(home, 1.1, 0xffffff);
  // 机头箭头：从载具出发指向前方，与路线箭头同一方向约定。
  path.add(
    new THREE.ArrowHelper(headingVector(vehicle.yaw_deg), home, 6, 0xe0685f, 1.8, 0.9)
  );

  view.radius = span * 2.2;
  // 视图以机头为上方：载具转向或移动时，整条路线在屏幕里跟着转。
  view.theta = -(Number(vehicle.yaw_deg) * Math.PI) / 180;
}

function update(snapshot) {
  if (renderer) draw(snapshot);
}

function fitSize() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width || canvas.height !== height) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

function applyCamera() {
  view.phi = Math.min(Math.max(view.phi, 0.12), Math.PI / 2 - 0.02);
  const phi = view.phi;
  camera.position.set(
    view.radius * Math.sin(phi) * Math.sin(view.theta),
    view.radius * Math.cos(phi),
    view.radius * Math.sin(phi) * Math.cos(view.theta)
  );
  camera.lookAt(0, 0, 0);
}

function bindPointer() {
  let dragging = false;
  let last = null;
  canvas.addEventListener("pointerdown", (event) => {
    dragging = true;
    last = [event.clientX, event.clientY];
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    view.phi -= (event.clientY - last[1]) * 0.008;
    last = [event.clientX, event.clientY];
  });
  for (const type of ["pointerup", "pointercancel", "pointerleave"]) {
    canvas.addEventListener(type, () => {
      dragging = false;
    });
  }
}

async function boot() {
  try {
    THREE = await fetchLibrary();
  } catch (error) {
    unavailable("库没有取到", error);
    return;
  }
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  } catch (error) {
    unavailable("浏览器没有可用的 WebGL", error);
    return;
  }
  renderer.setClearColor(0x12171d, 1);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  panel.hidden = false;
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(48, 1, 0.1, 8000);
  path = new THREE.Group();
  scene.add(path);
  bindPointer();
  fitSize();

  window.addEventListener("resize", fitSize);
  window.requestAnimationFrame(function frame() {
    window.requestAnimationFrame(frame);
    if (document.hidden) return;
    fitSize();
    applyCamera();
    renderer.render(scene, camera);
  });

  window.Trajectory3D = { update };
  // 请 flight.js 用它手里那份快照补画一次。
  if (typeof window.replayTrajectory === "function") window.replayTrajectory();
}

boot();
