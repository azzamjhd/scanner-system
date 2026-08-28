<script>
  import { onMount, onDestroy } from 'svelte';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import { livePointCloud, gantryPosition } from '../lib/ros.js';

  export let selectedPointIndices = [];
  export let detectedLandmarks = [];

  let canvasContainer;
  let scene, camera, renderer, controls;
  let pointCloudObject = null;
  let landmarkSpheres = [];
  let frameId;

  // Track position to update gantry position representation in 3D space
  let currentPos = { x: 0, y: 0, z: 0 };
  let gantryMesh;

  const unsubscribePos = gantryPosition.subscribe(value => {
    currentPos = value;
    if (gantryMesh) {
      gantryMesh.position.x = currentPos.x / 1000.0; // scale mm to meters
    }
  });

  let lastCloudUpdateTime = 0;
  const unsubscribeCloud = livePointCloud.subscribe(pcdMsg => {
    if (!pcdMsg || !scene) return;
    const now = performance.now();
    if (now - lastCloudUpdateTime < 200) return; // Limit geometry rebuilds to max 5 Hz
    lastCloudUpdateTime = now;
    updatePointCloud(pcdMsg);
  });

  function updatePointCloud(msg) {
    // rosbridge sends PointCloud2.data as a base64-encoded string, not a byte array.
    // Decode it to a Uint8Array before wrapping in DataView.
    let rawBytes;
    if (typeof msg.data === 'string') {
      const binaryStr = atob(msg.data);
      const len = binaryStr.length;
      rawBytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        rawBytes[i] = binaryStr.charCodeAt(i);
      }
    } else {
      rawBytes = new Uint8Array(msg.data);
    }

    const dataView = new DataView(rawBytes.buffer);
    const pointStep = msg.point_step;
    const totalPoints = Math.floor(rawBytes.byteLength / pointStep);

    // Dynamic stride for bandwidth/mobile optimization:
    // Over WAN/Tailscale, sample points to keep array allocations & loop iterations minimal.
    // If totalPoints > 20000, take every 2nd or 3rd point.
    const stepRatio = totalPoints > 40000 ? 3 : (totalPoints > 20000 ? 2 : 1);

    const positions = [];
    const colors = [];

    // Retrieve field offsets
    let xOffset = 0, yOffset = 4, zOffset = 8, rgbOffset = -1;
    for (let f of msg.fields) {
      if (f.name === 'x') xOffset = f.offset;
      if (f.name === 'y') yOffset = f.offset;
      if (f.name === 'z') zOffset = f.offset;
      if (f.name === 'rgb') rgbOffset = f.offset;
    }

    for (let i = 0; i < totalPoints; i += stepRatio) {
      const base = i * pointStep;
      if (base + zOffset + 4 > rawBytes.byteLength) break;

      const x = dataView.getFloat32(base + xOffset, true);
      const y = dataView.getFloat32(base + yOffset, true);
      const z = dataView.getFloat32(base + zOffset, true);

      // Skip NaN points (common in sparse scans)
      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

      positions.push(x, y, z);

      if (rgbOffset !== -1 && base + rgbOffset + 4 <= rawBytes.byteLength) {
        const rgbVal = dataView.getUint32(base + rgbOffset, true);
        const r = ((rgbVal >> 16) & 0xff) / 255.0;
        const g = ((rgbVal >> 8) & 0xff) / 255.0;
        const b = (rgbVal & 0xff) / 255.0;
        colors.push(r, g, b);
      } else {
        colors.push(0.5, 0.7, 1.0);
      }
    }

    // Reuse pointCloudObject geometry and material to prevent V8 GC / GPU memory leaks
    if (!pointCloudObject) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

      const material = new THREE.PointsMaterial({
        size: 0.01,
        vertexColors: true,
        // transparent: true,
        opacity: 0.8
      });

      pointCloudObject = new THREE.Points(geometry, material);
      scene.add(pointCloudObject);
    } else {
      const geom = pointCloudObject.geometry;
      geom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geom.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
      geom.attributes.position.needsUpdate = true;
      geom.attributes.color.needsUpdate = true;
      geom.computeBoundingSphere();
    }
  }

  // Monitor landmark changes to render interactive spheres
  $: if (detectedLandmarks && scene) {
    renderLandmarks(detectedLandmarks);
  }

  function renderLandmarks(landmarks) {
    // Clear old spheres and dispose WebGL resources
    landmarkSpheres.forEach(s => {
      scene.remove(s);
      if (s.geometry) s.geometry.dispose();
      if (s.material) s.material.dispose();
    });
    landmarkSpheres = [];

    landmarks.forEach((pt, index) => {
      const geom = new THREE.SphereGeometry(0.04, 16, 16); // Reduced segments for memory optimization
      const isSelected = selectedPointIndices.includes(index);
      const mat = new THREE.MeshBasicMaterial({
        color: isSelected ? 0x27ae60 : 0x7f8c8d // Green for selected, Gray/Silver otherwise
      });

      const mesh = new THREE.Mesh(geom, mat);
      mesh.position.set(pt.x, pt.y, pt.z);
      mesh.userData = { landmarkIndex: index };

      scene.add(mesh);
      landmarkSpheres.push(mesh);
    });
  }

  let pointerStartPos = { x: 0, y: 0 };

  function handlePointerDown(event) {
    const touch = event.touches ? event.touches[0] : event;
    pointerStartPos = { x: touch.clientX, y: touch.clientY };
  }

  function handleClick(event) {
    if (!renderer || !camera) return;

    const touch = event.changedTouches ? event.changedTouches[0] : event;
    const clientX = touch ? touch.clientX : event.clientX;
    const clientY = touch ? touch.clientY : event.clientY;

    const dx = clientX - pointerStartPos.x;
    const dy = clientY - pointerStartPos.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // Only trigger landmark raycast if it was a tap/click without dragging (dist <= 10px)
    if (dist > 10) return;

    const rect = renderer.domElement.getBoundingClientRect();
    const relX = clientX - rect.left;
    const relY = clientY - rect.top;

    const mouse = new THREE.Vector2(
      (relX / rect.width) * 2 - 1,
      -(relY / rect.height) * 2 + 1
    );

    const raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.05; // 5cm target margins
    raycaster.setFromCamera(mouse, camera);

    const intersects = raycaster.intersectObjects(landmarkSpheres);
    if (intersects.length > 0) {
      const hitIndex = intersects[0].object.userData.landmarkIndex;
      // Toggle selection list state
      if (selectedPointIndices.includes(hitIndex)) {
        selectedPointIndices = selectedPointIndices.filter(val => val !== hitIndex);
      } else {
        selectedPointIndices = [...selectedPointIndices, hitIndex];
      }
      renderLandmarks(detectedLandmarks);
    }
  }

  export function resetCamera() {
    if (!camera || !controls) return;
    camera.position.set(0.825, 0, 1.2);
    controls.target.set(0.825, 0, 0);
    controls.update();
  }

  onMount(() => {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0e0e0e);
    
    camera = new THREE.PerspectiveCamera(60, canvasContainer.clientWidth / canvasContainer.clientHeight, 0.1, 100);
    // camera = new THREE.OrthographicCamera(
    //   canvasContainer.clientWidth / -500, 
    //   canvasContainer.clientWidth / 500, 
    //   canvasContainer.clientHeight / 500, 
    //   canvasContainer.clientHeight / -500, 
    //   0.1, 
    //   100
    // );
    camera.position.set(0.825, 0, 1.2);
    camera.lookAt(0.825, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    canvasContainer.appendChild(renderer.domElement);

    // OrbitControls setup
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = true;
    controls.maxPolarAngle = Math.PI + 0.1;
    controls.target.set(0.825, 0, 0);
    controls.update();

    // Bed plane visualization grid
    const grid = new THREE.GridHelper(2.0, 10, 0x34495e, 0x1d2731);
    grid.rotation.x = Math.PI / 2;
    grid.position.x = 1.0;
    // scene.add(grid);

    // Gantry carriage physical object mock representation
    const gantryGeom = new THREE.BoxGeometry(0.01, 0.8, 0.5);
    gantryGeom.translate(0, 0, 0.25); // Center the box on the X-axis
    const gantryMat = new THREE.MeshBasicMaterial({ color: 0x3498db, wireframe: false, transparent: true, opacity: 0.4 });
    gantryMesh = new THREE.Mesh(gantryGeom, gantryMat);
    scene.add(gantryMesh);

    const animate = () => {
      frameId = requestAnimationFrame(animate);
      if (controls) controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // Listen to resize
    const handleResize = () => {
      if (!canvasContainer || !camera || !renderer) return;
      camera.aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    };
    window.addEventListener('resize', handleResize);
  });

  onDestroy(() => {
    cancelAnimationFrame(frameId);
    window.removeEventListener('resize', handleResize);
    unsubscribePos();
    unsubscribeCloud();
    if (pointCloudObject) {
      if (pointCloudObject.geometry) pointCloudObject.geometry.dispose();
      if (pointCloudObject.material) pointCloudObject.material.dispose();
    }
    landmarkSpheres.forEach(s => {
      if (s.geometry) s.geometry.dispose();
      if (s.material) s.material.dispose();
    });
    landmarkSpheres = [];
    if (controls) controls.dispose();
    if (renderer) renderer.dispose();
  });
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<div 
  bind:this={canvasContainer} 
  class="canvas-wrapper" 
  on:pointerdown={handlePointerDown}
  on:click={handleClick}
>
  <button class="reset-cam-btn" on:click|stopPropagation={resetCamera} title="Reset 3D View">
    🎯 Reset View
  </button>
</div>

<style>
  .canvas-wrapper {
    width: 100%;
    height: 100%;
    position: relative;
    border-radius: 12px;
    border: 1px solid #2d2d2d;
    box-shadow: inset 0 0 10px rgba(0,0,0,0.5);
    overflow: hidden;
  }

  .reset-cam-btn {
    position: absolute;
    top: 12px;
    right: 12px;
    background: rgba(30, 30, 30, 0.85);
    color: #e0e0e0;
    border: 1px solid #444;
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 0.85rem;
    cursor: pointer;
    backdrop-filter: blur(4px);
    transition: background 0.2s, border-color 0.2s;
    z-index: 10;
  }

  .reset-cam-btn:hover {
    background: rgba(50, 50, 50, 0.95);
    border-color: #3498db;
    color: #ffffff;
  }
</style>