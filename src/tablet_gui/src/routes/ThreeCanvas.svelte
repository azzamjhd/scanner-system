<script>
  import { onMount, onDestroy } from 'svelte';
  import * as THREE from 'three';
  import { livePointCloud, gantryPosition } from '../lib/ros.js';

  export let selectedPointIndices = [];
  export let detectedLandmarks = [];

  let canvasContainer;
  let scene, camera, renderer;
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

  const unsubscribeCloud = livePointCloud.subscribe(pcdMsg => {
    if (!pcdMsg || !scene) return;
    updatePointCloud(pcdMsg);
  });

  function updatePointCloud(msg) {
    if (pointCloudObject) {
      scene.remove(pointCloudObject);
    }

    // Parse standard PointCloud2 binary data
    const geometry = new THREE.BufferGeometry();
    const positions = [];
    const colors = [];

    // Fields parser: x, y, z are float32
    // Inside ROS PointCloud2 message format: data is uint8 array
    const dataView = new DataView(new Uint8Array(msg.data).buffer);
    const pointStep = msg.point_step;
    const numPoints = msg.data.length / pointStep;

    // Retrieve offsets
    let xOffset = 0, yOffset = 4, zOffset = 8, rgbOffset = -1;
    for (let f of msg.fields) {
      if (f.name === 'x') xOffset = f.offset;
      if (f.name === 'y') yOffset = f.offset;
      if (f.name === 'z') zOffset = f.offset;
      if (f.name === 'rgb') rgbOffset = f.offset;
    }

    for (let i = 0; i < numPoints; i++) {
      const base = i * pointStep;
      if (base + 12 > msg.data.length) break;

      const x = dataView.getFloat32(base + xOffset, true);
      const y = dataView.getFloat32(base + yOffset, true);
      const z = dataView.getFloat32(base + zOffset, true);

      positions.push(x, y, z);

      if (rgbOffset !== -1 && base + rgbOffset + 4 <= msg.data.length) {
        // Extract color components
        const rgbVal = dataView.getUint32(base + rgbOffset, true);
        const r = ((rgbVal >> 16) & 0xff) / 255.0;
        const g = ((rgbVal >> 8) & 0xff) / 255.0;
        const b = (rgbVal & 0xff) / 255.0;
        colors.push(r, g, b);
      } else {
        colors.push(0.5, 0.7, 1.0); // Default blueish tint
      }
    }

    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

    const material = new THREE.PointsMaterial({
      size: 0.015,
      vertexColors: true,
      transparent: true,
      opacity: 0.8
    });

    pointCloudObject = new THREE.Points(geometry, material);
    scene.add(pointCloudObject);
  }

  // Monitor landmark changes to render interactive spheres
  $: if (detectedLandmarks && scene) {
    renderLandmarks(detectedLandmarks);
  }

  function renderLandmarks(landmarks) {
    // Clear old spheres
    landmarkSpheres.forEach(s => scene.remove(s));
    landmarkSpheres = [];

    landmarks.forEach((pt, index) => {
      const geom = new THREE.SphereGeometry(0.04, 32, 32); // 4cm radius sphere
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

  function handleTouch(event) {
    if (!renderer || !camera) return;

    // Handle touch coordinate mapping
    const rect = renderer.domElement.getBoundingClientRect();
    const touch = event.touches ? event.touches[0] : event;
    const clientX = touch.clientX - rect.left;
    const clientY = touch.clientY - rect.top;

    const mouse = new THREE.Vector2(
      (clientX / rect.width) * 2 - 1,
      -(clientY / rect.height) * 2 + 1
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

  onMount(() => {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0e0e0e);

    camera = new THREE.PerspectiveCamera(60, canvasContainer.clientWidth / canvasContainer.clientHeight, 0.1, 100);
    camera.position.set(0, -1.8, 1.8);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    canvasContainer.appendChild(renderer.domElement);

    // Bed plane visualization grid
    const grid = new THREE.GridHelper(3.0, 30, 0x34495e, 0x1d2731);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);

    // Gantry carriage physical object mock representation
    const gantryGeom = new THREE.BoxGeometry(0.1, 0.5, 0.1);
    const gantryMat = new THREE.MeshBasicMaterial({ color: 0x9b59b6, wireframe: true });
    gantryMesh = new THREE.Mesh(gantryGeom, gantryMat);
    scene.add(gantryMesh);

    const animate = () => {
      frameId = requestAnimationFrame(animate);
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
    unsubscribePos();
    unsubscribeCloud();
    if (renderer) renderer.dispose();
  });
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<div 
  bind:this={canvasContainer} 
  class="canvas-wrapper" 
  on:touchstart={handleTouch}
  on:mousedown={handleTouch}
></div>

<style>
  .canvas-wrapper {
    width: 100%;
    height: 100%;
    position: relative;
    border-radius: 12px;
    border: 1px solid #2d2d2d;
    box-shadow: inset 0 0 10px rgba(0,0,0,0.5);
  }
</style>
