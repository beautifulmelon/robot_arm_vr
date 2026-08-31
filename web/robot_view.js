/**
 * 로봇 3D 표시 — VR(index.html)과 Mac 대시보드(dashboard.html)가 공유한다.
 *
 * 기구학은 전부 서버(placo)에 있다. 여기서는 서버가 보낸 링크 pose 를 받아
 * 그 자리에 상자를 놓기만 한다. 두 화면이 같은 코드를 쓰므로 한쪽만 고쳐져
 * 서로 다르게 보이는 일이 없다.
 *
 * 서버가 보내는 pose 는 **로봇 좌표계** 그대로다. 뷰 변환(제어 매핑의 역행렬)은
 * state.view_matrix 로 따로 와서 루트 그룹에 한 번만 걸린다. 링크마다 걸면 링크
 * 로컬 오프셋(박스 중심, 메시 원점)이 함께 변환되지 않아 형상이 흩어진다.
 *
 * 형상은 두 가지 모드가 있다.
 *   박스   기본. 바운딩 박스만 그린다. 가볍고 관절 구조가 잘 보인다.
 *   메시   실제 STL. 팔+손 331k 삼각형 / 16 MB. Mac 은 여유롭고 Quest 는 부담.
 */

import * as THREE from "three";
import { STLLoader } from "/vendor/STLLoader.js";

const _stlLoader = new STLLoader();
const _stlCache = new Map();   // url -> Promise<BufferGeometry>. 같은 파일을 4번 받지 않게.

function loadSTL(url) {
  if (!_stlCache.has(url)) {
    _stlCache.set(url, new Promise((res, rej) => _stlLoader.load(url, res, undefined, rej)));
  }
  return _stlCache.get(url);
}

export const COLORS = {
  base: 0x6e7681, ok: 0x3d8bfd, near: 0xd29922, limit: 0xf85149,
  ee: 0x3fb950, target: 0xa371f7, workspace: 0x30363d,
  ctrlIdle: 0x8b949e, ctrlActive: 0x3fb950,
};

export class RobotView {
  /**
   * @param {THREE.Object3D} parent 로봇을 담을 부모 (VR 은 stage 그룹, 데스크톱은 scene)
   * @param {object} opts
   *   showGrid    바닥 격자 표시
   *   showMarkers EE/목표/작업반경 표식 표시
   *   useMeshes   실제 STL 형상으로 그린다 (기본 false = 바운딩 박스)
   *               팔+손 합쳐 331k 삼각형 / 16 MB 라 Quest 에서는 부담이 된다.
   *               Mac 대시보드는 켜고, VR 은 필요할 때만 켜는 것을 권장.
   */
  constructor(parent, opts = {}) {
    this.parent = parent;
    this.opts = { showGrid: true, showMarkers: true, useMeshes: false, ...opts };

    this.linkNodes = new Map();
    this.builtSig = null;
    this.workspaceShell = null;

    this.root = new THREE.Group();
    // 본체(차체) 위에 팔이 놓인 모습을 보려면 팔 전체를 arm_mount 만큼 올려야
    // 한다. 링크 pose 는 팔 base_link 기준으로 오기 때문이다. 그래서 팔을
    // armGroup 으로 한 겹 감싸고 그 그룹만 옮긴다.
    // ★ 본체 모드가 꺼져 있으면 armGroup 오프셋이 0 이라 기존과 완전히 같다.
    this.armGroup = new THREE.Group();
    this.bodyGroup = new THREE.Group();
    this.bodyGroup.visible = false;
    // 뷰 변환은 여기에 한 번만 걸린다. 링크마다 걸면 로컬 오프셋(박스 중심,
    // 메시 원점)이 함께 변환되지 않아 형상이 흩어진다.
    this.root.matrixAutoUpdate = false;
    parent.add(this.root);
    // 바닥 격자와 본체는 root(=바닥 기준), 팔은 armGroup 에 들어간다.
    this.root.add(this.bodyGroup);
    this.root.add(this.armGroup);

    if (this.opts.showGrid) {
      const grid = new THREE.GridHelper(1.6, 16, 0x58a6ff, 0x30363d);
      // ★ GridHelper 는 three.js 관례(y-up)대로 자신의 XZ 평면에 격자를 그린다.
      //   이 그룹의 로컬 좌표는 로봇 좌표계(FLU, z-up)라 바닥은 XY 평면이다.
      //   그대로 두면 격자가 수직 벽이 된다(실측: 월드 x=0 평면에 놓임).
      //   X 축 +90° 회전으로 격자면을 XY 평면(로봇 z=0 = 팔이 놓인 면)에 맞춘다.
      grid.rotation.x = Math.PI / 2;
      grid.material.opacity = 0.35;
      grid.material.transparent = true;
      this.root.add(grid);
    }

    if (this.opts.showMarkers) {
      this.eeMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.022, 16, 12),
        new THREE.MeshStandardMaterial({
          color: COLORS.ee, emissive: COLORS.ee, emissiveIntensity: 0.4 }));
      this.armGroup.add(this.eeMarker);

      this.targetMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.028, 12, 8),
        new THREE.MeshBasicMaterial({ color: COLORS.target, wireframe: true }));
      this.armGroup.add(this.targetMarker);
    }
  }

  /** 서버가 보낸 정적 형상으로 링크 상자를 만든다 (형상이 바뀔 때만 재생성). */
  build(geometry) {
    const sig = JSON.stringify(geometry.map((g) => [g.name, g.size]));
    if (sig === this.builtSig) return;
    this.builtSig = sig;

    for (const { group } of this.linkNodes.values()) this.root.remove(group);
    this.linkNodes.clear();
    this.meshWanted = 0;    // 붙이려고 시도한 STL 수
    this.meshLoaded = 0;    // 실제로 붙은 수

    for (const g of geometry) {
      const group = new THREE.Group();
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2]),
        new THREE.MeshStandardMaterial({
          color: g.joint ? COLORS.ok : COLORS.base,
          roughness: 0.55, metalness: 0.15, transparent: true, opacity: 0.92,
          side: THREE.DoubleSide }));   // 미러(det=-1) 시 winding 이 뒤집힌다
      // 상자 중심은 링크 원점과 다르다. 링크 그룹의 자식으로 두면 회전을 자동으로 따라간다.
      mesh.position.set(g.center[0], g.center[1], g.center[2]);
      group.add(mesh);

      // 외곽선 — 상자만 있으면 자세를 읽기 어렵다
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(mesh.geometry),
        new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 }));
      edges.position.copy(mesh.position);
      group.add(edges);

      this.armGroup.add(group);
      const node = { group, mesh, edges, joint: g.joint, meshParts: [] };
      this.linkNodes.set(g.name, node);

      // 실제 형상은 비동기로 받아 붙인다. 다 받을 때까지는 박스가 보이므로
      // 화면이 비는 구간이 없다.
      if (this.opts.useMeshes && (g.meshes || []).length) {
        this.meshWanted += g.meshes.length;
        this._attachMeshes(node, g);
      }
    }
  }

  /** URDF visual 스펙대로 STL 을 붙인다. scale/origin 을 반드시 반영해야 한다
   *  (이 URDF 들은 mm 단위 메시가 섞여 있어 scale 을 빠뜨리면 1000배로 나온다). */
  async _attachMeshes(node, g) {
    const material = new THREE.MeshStandardMaterial({
      color: g.joint ? COLORS.ok : COLORS.base,
      roughness: 0.6, metalness: 0.2, side: THREE.DoubleSide });
    node.meshMaterial = material;

    for (const spec of g.meshes) {
      let geom;
      try {
        geom = await loadSTL(spec.url);
      } catch (e) {
        console.warn("STL 로드 실패", spec.url, e);
        continue;
      }
      if (!this.linkNodes.has(g.name)) return;   // 그 사이 재생성됐으면 버린다
      const m = new THREE.Mesh(geom, material);
      m.scale.set(spec.scale[0], spec.scale[1], spec.scale[2]);
      m.position.set(spec.xyz[0], spec.xyz[1], spec.xyz[2]);
      m.rotation.set(spec.rpy[0], spec.rpy[1], spec.rpy[2], "ZYX");  // URDF rpy 순서
      node.group.add(m);
      node.meshParts.push(m);
      this.meshLoaded += 1;
    }

    // 실제 형상이 하나라도 붙으면 대체용 박스는 감춘다
    if (node.meshParts.length) {
      node.mesh.visible = false;
      node.edges.visible = false;
    }
  }

  /** 형상 모드를 한 줄로. 헤드셋 안에서 메시가 정말 붙었는지 눈으로 확인용. */
  meshInfo() {
    if (!this.opts.useMeshes) return "박스";
    if (!this.meshWanted) return "박스(메시없음)";
    return `메시 ${this.meshLoaded}/${this.meshWanted}`;
  }

  /**
   * 본체(차체) 형상을 불러온다. scripts/09_body_visual.py 가 만든 정적 JSON.
   *
   * 본체는 관절이 없어 형상이 절대 안 바뀌므로 한 번만 받으면 된다.
   * /vendor 아래 두는 이유는 그쪽만 StaticFiles 로 통째 마운트돼 있어서다.
   * 서버 상태 패킷에 싣지 않는 이유도 그것이다 (매 프레임 보낼 값이 아니다).
   */
  async loadBody(url = "/vendor/body_geometry.json") {
    if (this.bodyLoaded) return true;
    let data;
    try {
      const res = await fetch(url);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      data = await res.json();
    } catch (e) {
      console.warn("[RobotView] 본체 형상을 못 불러왔습니다:", e.message);
      return false;
    }
    // 팔을 본체 위 제자리로 올리는 오프셋. 회전은 없다(기구 담당 검증).
    this.armMount = data.arm_mount || [0, 0, 0];

    const mat = new THREE.MeshStandardMaterial({
      color: COLORS.base, roughness: 0.85, metalness: 0.1,
      transparent: true, opacity: 0.55, side: THREE.DoubleSide,
    });
    for (const part of data.parts || []) {
      const box = new THREE.Mesh(
        new THREE.BoxGeometry(part.size[0], part.size[1], part.size[2]), mat);
      box.position.set(part.center[0], part.center[1], part.center[2]);
      if (part.quat) box.quaternion.set(...part.quat);
      this.bodyGroup.add(box);
      // 뼈대가 보이도록 모서리도 같이 (반투명이라 면만으로는 형태가 뭉갠다)
      const edge = new THREE.LineSegments(
        new THREE.EdgesGeometry(box.geometry),
        new THREE.LineBasicMaterial({ color: 0x8b949e, transparent: true, opacity: 0.5 }));
      edge.position.copy(box.position);
      edge.quaternion.copy(box.quaternion);
      this.bodyGroup.add(edge);
    }
    this.bodyLoaded = true;
    return true;
  }

  /**
   * 로봇(본체) 표시 on/off.
   *   true  = 로봇 O 모드 — 차체 위에 얹힌 실제 모습. 바닥이 진짜 바닥.
   *   false = 로봇 X 모드 — 팔만. base_link 가 원점 (지금까지의 화면).
   */
  setBodyVisible(on) {
    this.bodyGroup.visible = !!on;
    const m = on && this.armMount ? this.armMount : [0, 0, 0];
    this.armGroup.position.set(m[0], m[1], m[2]);
  }

  /** 매 프레임 서버 상태로 갱신. */
  update(s) {
    if (!s) return;

    // 로봇 좌표계(FLU) → 뷰 좌표계(WebXR RUB). 미러가 켜지면 det=-1 이 되는데
    // three.js 는 음수 스케일로 받아들이며, 면 winding 이 뒤집히므로 재질을
    // DoubleSide 로 둬야 안쪽이 뚫려 보이지 않는다 (build() 에서 설정).
    if (s.view_matrix && s.view_matrix.length === 16) {
      this.root.matrix.set(...s.view_matrix);
      this.root.matrixWorldNeedsUpdate = true;
    }

    if (s.geometry) this.build(s.geometry);

    const status = new Map((s.joints || []).map((j) => [j.name, j.status]));

    for (const l of s.links || []) {
      const node = this.linkNodes.get(l.name);
      if (!node) continue;
      node.group.position.set(l.p[0], l.p[1], l.p[2]);
      node.group.quaternion.set(l.q[0], l.q[1], l.q[2], l.q[3]);
      if (node.joint) {
        const st = status.get(node.joint) || "ok";
        const hex = st === "limit" ? COLORS.limit : st === "near" ? COLORS.near : COLORS.ok;
        node.mesh.material.color.setHex(hex);
        if (node.meshMaterial) node.meshMaterial.color.setHex(hex);
      }
    }

    if (!this.opts.showMarkers) return;

    if (s.ee_point) this.eeMarker.position.set(...s.ee_point);
    if (s.target_point) {
      this.targetMarker.position.set(...s.target_point);
      this.targetMarker.visible = true;
      // 목표에 도달 못 하고 있으면 눈에 띄게
      if (s.ik_err_mm != null) {
        this.targetMarker.material.color.setHex(
          s.ik_err_mm > 10 ? COLORS.limit : COLORS.target);
      }
    }

    if (s.workspace) {
      if (!this.workspaceShell) {
        this.workspaceShell = new THREE.Mesh(
          new THREE.SphereGeometry(1, 20, 14),
          new THREE.MeshBasicMaterial({
            color: COLORS.workspace, wireframe: true, transparent: true, opacity: 0.16 }));
        this.armGroup.add(this.workspaceShell);
      }
      this.workspaceShell.position.set(...s.workspace.origin);
      this.workspaceShell.scale.setScalar(s.workspace.max);
      this.workspaceShell.material.color.setHex(s.clamped ? COLORS.near : COLORS.workspace);
      this.workspaceShell.material.opacity = s.clamped ? 0.3 : 0.16;
    }
  }

  dispose() {
    this.parent.remove(this.root);
    this.linkNodes.clear();
    this.builtSig = null;
    this.workspaceShell = null;
  }
}

/** 두 화면이 같은 조명을 쓰도록. */
export function addLights(scene) {
  scene.add(new THREE.HemisphereLight(0xffffff, 0x404060, 2.2));
  const dir = new THREE.DirectionalLight(0xffffff, 1.6);
  dir.position.set(1, 3, 1);
  scene.add(dir);
}
