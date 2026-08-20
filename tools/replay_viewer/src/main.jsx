import React, { useEffect, useMemo, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import {
  Box, Camera, ChevronDown, CircleDot, Crosshair, Database,
  Gauge, Layers3, Maximize2, Pause, Play, RotateCcw, Settings2,
  SkipBack, SkipForward, Wifi, ZoomIn, ZoomOut
} from 'lucide-react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import './styles.css';
import './overrides.css';

const FALLBACK_DURATION = 46.429172;

function IconButton({ label, children, active = false, onClick }) {
  return <button className={`icon-button ${active ? 'active' : ''}`} title={label} aria-label={label} onClick={onClick}>{children}</button>;
}

function Badge({ tone = 'ok', children }) {
  return <span className={`badge ${tone}`}><i />{children}</span>;
}

function frameAt(frames, time) {
  if (!frames?.length || time < frames[0].time) return -1;
  let low = 0;
  let high = frames.length - 1;
  while (low <= high) {
    const middle = (low + high) >> 1;
    if (frames[middle].time <= time) low = middle + 1;
    else high = middle - 1;
  }
  return high;
}

function ReplayScene({ kind, manifest, time, assetBase }) {
  const mount = useRef(null);
  const state = useRef(null);

  useEffect(() => {
    if (!manifest) return undefined;
    let disposed = false;
    const host = mount.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(kind === 'global' ? 0x090b0d : 0x0c0f11);
    scene.fog = new THREE.FogExp2(0x090b0d, kind === 'global' ? 0.012 : 0.026);
    const camera = new THREE.PerspectiveCamera(48, 1, 0.05, 400);
    camera.position.set(kind === 'global' ? 13 : 8, kind === 'global' ? 10 : 5, kind === 'global' ? 15 : 9);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0, 0);
    controls.maxDistance = kind === 'global' ? 120 : 35;
    controls.minDistance = 1.5;
    const grid = new THREE.GridHelper(kind === 'global' ? 40 : 30, kind === 'global' ? 40 : 30, 0x273238, 0x182126);
    grid.position.y = kind === 'global' ? -0.4 : -2.0;
    scene.add(grid);
    const geometries = [];
    const materials = [];

    if (kind === 'local') {
      const maximum = Math.max(...manifest.lidarFrames.map(frame => frame.localCount));
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(maximum * 3), 3));
      geometry.setDrawRange(0, 0);
      const material = new THREE.PointsMaterial({ color: 0x49c8b5, size: 0.055, transparent: true, opacity: 0.88 });
      scene.add(new THREE.Points(geometry, material));
      geometries.push(geometry); materials.push(material);
      state.current = {
        geometry, lastFrame: -2, localData: null,
        update(nextTime) {
          this.time = nextTime;
          const index = frameAt(manifest.lidarFrames, nextTime);
          if (index === this.lastFrame || !this.localData) return;
          this.lastFrame = index;
          if (index < 0) { geometry.setDrawRange(0, 0); return; }
          const frame = manifest.lidarFrames[index];
          const source = this.localData.subarray(frame.localOffset * 3, (frame.localOffset + frame.localCount) * 3);
          geometry.attributes.position.array.set(source);
          geometry.attributes.position.needsUpdate = true;
          geometry.setDrawRange(0, frame.localCount);
          geometry.computeBoundingSphere();
        }
      };
      fetch(`${assetBase}/lidar-local.bin`).then(response => response.arrayBuffer()).then(buffer => {
        if (disposed) return;
        state.current.localData = new Float32Array(buffer);
        state.current.lastFrame = -2;
        state.current.update(state.current.time ?? 0);
      });
    } else {
      const geometry = new THREE.BufferGeometry();
      geometry.setDrawRange(0, 0);
      const material = new THREE.PointsMaterial({ color: 0x49c8b5, size: 0.048, transparent: true, opacity: 0.78 });
      scene.add(new THREE.Points(geometry, material));
      geometries.push(geometry); materials.push(material);
      const trajectoryPoints = manifest.trajectory.map(item => new THREE.Vector3(item[1], item[2], item[3]));
      const poseFrames = manifest.trajectory.map(item => ({ time: item[0] }));
      const trajectoryGeometry = new THREE.BufferGeometry().setFromPoints(trajectoryPoints);
      trajectoryGeometry.setDrawRange(0, 0);
      const trajectoryMaterial = new THREE.LineBasicMaterial({ color: 0xffc857 });
      scene.add(new THREE.Line(trajectoryGeometry, trajectoryMaterial));
      geometries.push(trajectoryGeometry); materials.push(trajectoryMaterial);
      const markerGeometry = new THREE.SphereGeometry(0.18, 16, 16);
      const markerMaterial = new THREE.MeshBasicMaterial({ color: 0xffc857 });
      const marker = new THREE.Mesh(markerGeometry, markerMaterial);
      marker.visible = false;
      scene.add(marker);
      geometries.push(markerGeometry); materials.push(markerMaterial);
      state.current = {
        geometry, trajectoryGeometry, marker, mapReady: false, lastScan: -2, lastPose: -2,
        update(nextTime) {
          this.time = nextTime;
          const scanIndex = frameAt(manifest.lidarFrames, nextTime);
          if (scanIndex !== this.lastScan && this.mapReady) {
            this.lastScan = scanIndex;
            const visiblePoints = scanIndex < 0 ? 0 : manifest.lidarFrames[scanIndex].mapOffset + manifest.lidarFrames[scanIndex].mapCount;
            geometry.setDrawRange(0, visiblePoints);
          }
          const poseIndex = frameAt(poseFrames, nextTime);
          if (poseIndex !== this.lastPose) {
            this.lastPose = poseIndex;
            trajectoryGeometry.setDrawRange(0, Math.max(0, poseIndex + 1));
            marker.visible = poseIndex >= 0;
            if (poseIndex >= 0) marker.position.copy(trajectoryPoints[poseIndex]);
          }
        }
      };
      if (manifest.mapAvailable) {
        fetch(`${assetBase}/lidar-map.bin`).then(response => response.arrayBuffer()).then(buffer => {
          if (disposed) return;
          geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buffer), 3));
          geometry.computeBoundingSphere();
          state.current.mapReady = true;
          state.current.lastScan = -2;
          state.current.update(state.current.time ?? 0);
        });
      }
    }

    const resize = () => {
      const { width, height } = host.getBoundingClientRect();
      renderer.setSize(width, height, false);
      camera.aspect = width / Math.max(1, height);
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(host);
    let animationFrame;
    const render = () => {
      controls.update();
      renderer.render(scene, camera);
      animationFrame = requestAnimationFrame(render);
    };
    resize(); render();
    return () => {
      disposed = true;
      cancelAnimationFrame(animationFrame); observer.disconnect(); controls.dispose();
      geometries.forEach(item => item.dispose()); materials.forEach(item => item.dispose());
      renderer.dispose(); host.removeChild(renderer.domElement); state.current = null;
    };
  }, [kind, manifest, assetBase]);

  useEffect(() => { state.current?.update(time); }, [time]);
  return <div ref={mount} className="three-mount" />;
}

function PanelHeader({ icon, title, topic, children }) {
  return <div className="panel-header"><div className="panel-title">{icon}<strong>{title}</strong><span>{topic}</span></div><div className="panel-tools">{children}</div></div>;
}

function formatClock(seconds) {
  return `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${(seconds % 60).toFixed(2).padStart(5, '0')}`;
}

function App() {
  const [catalog, setCatalog] = useState([]);
  const [datasetId, setDatasetId] = useState('m2dgr-anomaly');
  const [manifest, setManifest] = useState(null);
  const [playing, setPlaying] = useState(true);
  const [time, setTime] = useState(0);
  const [cameraMode, setCameraMode] = useState('RGB');
  const rgbVideo = useRef(null);
  const depthVideo = useRef(null);
  const assetBase = `/replay/${datasetId}`;
  const duration = manifest?.duration ?? FALLBACK_DURATION;
  const progress = Math.min(1, time / duration);

  useEffect(() => { fetch('/replay/catalog.json').then(response => response.json()).then(data => setCatalog(data.datasets)); }, []);
  useEffect(() => {
    setPlaying(false);
    setTime(0);
    setManifest(null);
    setCameraMode('RGB');
    fetch(`${assetBase}/manifest.json`).then(response => response.json()).then(data => {
      setManifest(data);
      setPlaying(true);
    });
  }, [assetBase]);
  useEffect(() => {
    const videos = [rgbVideo.current, depthVideo.current].filter(Boolean);
    videos.forEach(video => { video.playbackRate = 1; });
    if (playing) Promise.all(videos.map(video => video.play())).catch(() => setPlaying(false));
    else videos.forEach(video => video.pause());
  }, [playing, manifest]);
  useEffect(() => {
    let frame;
    const tick = () => {
      const video = rgbVideo.current;
      if (video && !video.paused) {
        setTime(video.currentTime);
        if (depthVideo.current && Math.abs(depthVideo.current.currentTime - video.currentTime) > 0.12) depthVideo.current.currentTime = video.currentTime;
      }
      frame = requestAnimationFrame(tick);
    };
    tick();
    return () => cancelAnimationFrame(frame);
  }, []);

  const seek = nextTime => {
    const bounded = Math.max(0, Math.min(duration, nextTime));
    [rgbVideo.current, depthVideo.current].forEach(video => { if (video) video.currentTime = bounded; });
    setTime(bounded);
  };
  const scanIndex = frameAt(manifest?.lidarFrames, time);
  const scan = scanIndex >= 0 ? manifest.lidarFrames[scanIndex] : null;
  const poseFrames = useMemo(() => manifest?.trajectory.map(item => ({ time: item[0] })) ?? [], [manifest]);
  const poseIndex = frameAt(poseFrames, time);
  const pose = poseIndex >= 0 ? manifest.trajectory[poseIndex] : [0, 0, 0, 0];
  const globalPoints = manifest?.mapAvailable && scan ? scan.mapOffset + scan.mapCount : 0;
  const sourceRows = Object.entries(manifest?.counts ?? {});
  const rateFor = name => {
    const rates = manifest?.id === 'r3live-degenerate-02'
      ? { LiDAR: '10.0 Hz', IMU: '204 Hz', RGB: '32.8 Hz', GNSS: '50.0 Hz' }
      : { LiDAR: '5.02 Hz', IMU: '100 Hz', RGB: '15.0 Hz', Depth: '15.0 Hz', Odometry: '20.0 Hz' };
    return rates[name] ?? '—';
  };

  return <main className="app-shell">
    <header className="topbar">
      <div className="brand"><span className="brand-mark"><Layers3 size={17} /></span><div><strong>UltraFusion Replay</strong><span>多源融合数据集播放器</span></div></div>
      <label className="dataset-select"><Database size={15} /><div><span>DATASET</span><select value={datasetId} onChange={event => setDatasetId(event.target.value)}>{catalog.map(item => <option key={item.id} value={item.id}>{item.name}</option>)}</select></div><ChevronDown size={15} /></label>
      <div className="run-state"><Badge>{manifest ? '数据已就绪' : '正在载入'}</Badge><span className="divider" /><span>完整回放 · {duration.toFixed(2)} s</span><span>{manifest?.mapAvailable ? '增量地图' : '原始数据模式'}</span><IconButton label="设置"><Settings2 size={17} /></IconButton></div>
    </header>

    <section className="workspace">
      <div className="upper-grid">
        <article className="viewer-panel local-panel">
          <PanelHeader icon={<CircleDot size={15} />} title="局部 LiDAR" topic={manifest?.topics?.lidar ?? 'loading'}>
            <Badge>{scanIndex >= 0 ? `${scanIndex + 1} / ${manifest?.lidarFrames.length}` : '等待首帧'}</Badge><IconButton label="重置视角"><RotateCcw size={15} /></IconButton><IconButton label="全屏"><Maximize2 size={15} /></IconButton>
          </PanelHeader>
          <div className="viewer-body"><ReplayScene kind="local" manifest={manifest} time={time} assetBase={assetBase} /><div className="axis-widget"><b className="x">X</b><b className="y">Y</b><b className="z">Z</b></div><div className="viewer-readout"><span>POINTS <b>{scan?.localCount.toLocaleString() ?? 0}</b></span><span>RANGE <b>0–90 m</b></span><span>FRAME <b>{manifest?.id === 'r3live-degenerate-02' ? 'camera_init' : 'rslidar'}</b></span></div></div>
        </article>

        <article className="viewer-panel camera-panel">
          <PanelHeader icon={<Camera size={15} />} title="相机" topic={cameraMode === 'RGB' ? manifest?.topics?.camera : manifest?.topics?.depth}>
            <div className="segmented"><button className={cameraMode === 'RGB' ? 'selected' : ''} onClick={() => setCameraMode('RGB')}>RGB</button><button disabled={!manifest?.depthFrames} className={cameraMode === 'Depth' ? 'selected' : ''} onClick={() => setCameraMode('Depth')}>Depth</button></div><IconButton label="全屏"><Maximize2 size={15} /></IconButton>
          </PanelHeader>
          <div className="camera-feed replay-video"><video key={`${datasetId}-rgb`} ref={rgbVideo} className={cameraMode === 'RGB' ? 'visible' : ''} src={`${assetBase}/camera-rgb.webm`} muted playsInline onEnded={() => { setPlaying(false); setTime(duration); }} />{manifest?.depthFrames > 0 && <video key={`${datasetId}-depth`} ref={depthVideo} className={cameraMode === 'Depth' ? 'visible' : ''} src={`${assetBase}/camera-depth.webm`} muted playsInline />}<div className="camera-readout"><span>{manifest?.id === 'r3live-degenerate-02' ? '1280 × 1024' : '640 × 480'}</span><span>{manifest?.cameraFps.toFixed(2)} Hz</span><Badge>{Math.min(manifest?.rgbFrames ?? 0, Math.floor(time * (manifest?.cameraFps ?? 1)) + 1)} / {manifest?.rgbFrames ?? 0}</Badge></div></div>
        </article>
      </div>

      <div className="lower-grid">
        <article className="global-view">
          <PanelHeader icon={<Box size={15} />} title={manifest?.mapAvailable ? '增量全局点云与轨迹' : '全局地图'} topic={manifest?.topics?.pose ?? 'unavailable'}>
            {manifest?.mapAvailable && <div className="legend"><span><i className="map-dot"/>已建地图</span><span><i className="traj-line"/>当前轨迹</span></div>}<IconButton label="放大"><ZoomIn size={15} /></IconButton><IconButton label="缩小"><ZoomOut size={15} /></IconButton><IconButton label="定位载体"><Crosshair size={15} /></IconButton><IconButton label="全屏"><Maximize2 size={15} /></IconButton>
          </PanelHeader>
          <div className="global-canvas"><ReplayScene kind="global" manifest={manifest} time={time} assetBase={assetBase} />{manifest?.mapAvailable ? <div className="pose-readout"><span>POSITION</span><strong>{pose.slice(1).map(value => value.toFixed(2)).join('  ')} m</strong><span>MAP POINTS</span><strong>{globalPoints.toLocaleString()}</strong></div> : <div className="map-unavailable"><strong>无可用位姿</strong><span>{manifest?.mapReason}</span><small>仅播放原始相机与当前 LiDAR 扫描</small></div>}</div>
        </article>

        <aside className="telemetry">
          <div className="telemetry-head"><Gauge size={15}/><strong>回放状态</strong><Badge>{playing ? 'PLAYING' : 'PAUSED'}</Badge></div>
          <dl className="metric-list"><div><dt>数据时间</dt><dd>{time.toFixed(2)} <small>s</small></dd></div><div><dt>完成度</dt><dd>{(progress * 100).toFixed(1)} <small>%</small></dd></div><div><dt>LiDAR 帧</dt><dd>{Math.max(0, scanIndex + 1)} <small>/ {manifest?.lidarFrames.length ?? 0}</small></dd></div><div><dt>地图点数</dt><dd>{globalPoints.toLocaleString()}</dd></div></dl>
          <div className="source-table"><div className="source-row head"><span>数据源</span><span>频率</span><span>总帧</span></div>{sourceRows.map(([name, count]) => <div className="source-row" key={name}><span><i className="source-ok"/>{name}</span><b>{rateFor(name)}</b><em>{count.toLocaleString()}</em></div>)}</div>
          <div className="resource-strip"><span>MODE <b>SEEKABLE</b></span><span>MAP <b>{manifest?.mapAvailable ? 'INCREMENTAL' : 'UNAVAILABLE'}</b></span></div>
        </aside>
      </div>
    </section>

    <footer className="transport">
      <div className="transport-buttons"><IconButton label="回到起点" onClick={() => { setPlaying(false); seek(0); }}><SkipBack size={17}/></IconButton><button className="play-button" aria-label={playing ? '暂停' : '播放'} onClick={() => { if (!playing && time >= duration - 0.05) seek(0); setPlaying(!playing); }}>{playing ? <Pause size={20} fill="currentColor"/> : <Play size={20} fill="currentColor"/>}</button><IconButton label="前进一帧" onClick={() => { setPlaying(false); seek(time + 1 / 15); }}><SkipForward size={17}/></IconButton></div>
      <time>{formatClock(time)}</time><div className="timeline"><input type="range" min="0" max={duration} step=".001" value={time} style={{'--progress': `${progress * 100}%`}} onPointerDown={() => setPlaying(false)} onChange={event => seek(Number(event.target.value))}/><div className="events"><i style={{left:'12%'}}/><i style={{left:'46%'}}/><i className="warn" style={{left:'71%'}}/></div></div><time className="duration">{formatClock(duration)}</time><button className="speed">1.0× <ChevronDown size={13}/></button><span className="clock-sync"><Wifi size={14}/> BAG TIME</span>
    </footer>
  </main>;
}

createRoot(document.getElementById('root')).render(<App />);
