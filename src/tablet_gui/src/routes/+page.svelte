<script>
  import { onMount } from 'svelte';
  import { 
    initRos, 
    rosConnected, 
    gantryPosition, 
    scanLogs, 
    publishSoftAbort, 
    triggerScanBody,
    triggerExecuteMassage
  } from '../lib/ros.js';
  import ThreeCanvas from './ThreeCanvas.svelte';

  // Config parameters
  let bridgeUrl = 'ws://localhost:9090';
  let startMm = 0.0;
  let endMm = 600.0;
  let speedMms = 35.0;

  let forceLimitN = 15.0;
  let massagePattern = 'linear';

  let detectedLandmarks = [];
  let selectedPointIndices = [];
  
  let scanGoal = null;
  let executionGoal = null;

  let isScanning = false;
  let isExecuting = false;

  let currentPos = { x: 0, y: 0, z: 0 };
  let isConnected = false;

  rosConnected.subscribe(val => isConnected = val);
  gantryPosition.subscribe(val => currentPos = val);

  onMount(() => {
    // Attempt auto-connection
    initRos(bridgeUrl);
  });

  function startScanSeq() {
    isScanning = true;
    scanGoal = triggerScanBody(
      startMm, 
      endMm, 
      speedMms,
      (feedback) => {
        // Feedback callback
      },
      (result) => {
        isScanning = false;
        if (result.success) {
          detectedLandmarks = result.detected_landmarks || [];
          selectedPointIndices = detectedLandmarks.map((_, i) => i); // Auto-select all by default
        }
      }
    );
  }

  function cancelScanSeq() {
    if (scanGoal) {
      scanGoal.cancel();
      isScanning = false;
    }
  }

  function startMassagePlan() {
    // Filter coordinates of selected landmarks
    const targets = detectedLandmarks.filter((_, index) => selectedPointIndices.includes(index));
    if (targets.length === 0) {
      alert("Please select at least one massage point!");
      return;
    }
    
    isExecuting = true;
    executionGoal = triggerExecuteMassage(
      targets,
      forceLimitN,
      massagePattern,
      (feedback) => {
        // Active feedback processing
      },
      (result) => {
        isExecuting = false;
      }
    );
  }

  function cancelMassagePlan() {
    if (executionGoal) {
      executionGoal.cancel();
      isExecuting = false;
    }
  }
</script>

<div class="app-layout">
  <!-- Sidebar Panel -->
  <aside class="control-sidebar">
    <h2>Control Panel</h2>
    
    <!-- Connection State -->
    <div class="card status-card">
      <div class="row align-center">
        <span class="status-indicator {isConnected ? 'connected' : 'disconnected'}"></span>
        <span>{isConnected ? 'Bridge Connected' : 'Bridge Disconnected'}</span>
      </div>
      {#if !isConnected}
        <div class="form-group margin-top-xs">
          <input type="text" bind:value={bridgeUrl} />
          <button class="btn btn-connect" on:click={() => initRos(bridgeUrl)}>Connect</button>
        </div>
      {/if}
    </div>

    <!-- Scan Path Specs -->
    <div class="card spec-card">
      <h3>Scan Setup</h3>
      <div class="form-group">
        <label>Start Pos (mm): {startMm}</label>
        <input type="range" min="0" max="1500" step="50" bind:value={startMm} disabled={isScanning} />
      </div>
      <div class="form-group">
        <label>End Pos (mm): {endMm}</label>
        <input type="range" min="100" max="2000" step="50" bind:value={endMm} disabled={isScanning} />
      </div>
      <div class="form-group">
        <label>Speed (mm/s): {speedMms}</label>
        <input type="range" min="10" max="100" step="5" bind:value={speedMms} disabled={isScanning} />
      </div>

      {#if !isScanning}
        <button class="btn btn-primary" on:click={startScanSeq} disabled={!isConnected}>Start Body Scan</button>
      {:else}
        <button class="btn btn-warning" on:click={cancelScanSeq}>Cancel Scan</button>
      {/if}
    </div>

    <!-- Massage Plan Setup -->
    <div class="card plan-card">
      <h3>Massage Settings</h3>
      <div class="form-group">
        <label>Force Target (N): {forceLimitN} N</label>
        <input type="range" min="5" max="30" step="1" bind:value={forceLimitN} disabled={isExecuting} />
      </div>
      <div class="form-group">
        <label>Pattern Mode</label>
        <select bind:value={massagePattern} disabled={isExecuting}>
          <option value="linear">Direct/Linear</option>
          <option value="circular">Circular Spiral</option>
          <option value="cross-fiber">Cross-Fiber</option>
        </select>
      </div>

      {#if !isExecuting}
        <button 
          class="btn btn-success" 
          on:click={startMassagePlan} 
          disabled={!isConnected || detectedLandmarks.length === 0}
        >
          Execute Plan ({selectedPointIndices.length})
        </button>
      {:else}
        <button class="btn btn-warning" on:click={cancelMassagePlan}>Pause/Halt Execution</button>
      {/if}
    </div>

    <!-- Software Abort -->
    <button class="btn btn-danger abort-button" on:click={publishSoftAbort}>
      SOFTWARE ABORT
    </button>
  </aside>

  <!-- Large Viewport Canvas and Logger -->
  <main class="viewport-main">
    <header class="bar-header">
      <div class="telemetry-block">
        <span>Gantry Position: <strong>X: {currentPos.x.toFixed(1)} mm</strong></span>
      </div>
    </header>

    <div class="visualizer-container">
      <ThreeCanvas bind:selectedPointIndices bind:detectedLandmarks />
    </div>

    <section class="logs-console">
      <h4>System Notifications</h4>
      <div class="logs-feed">
        {#each $scanLogs as log}
          <div class="log-line">{log}</div>
        {/each}
      </div>
    </section>
  </main>
</div>

<style>
  .app-layout {
    display: flex;
    flex-direction: row;
    width: 100vw;
    height: 100vh;
    box-sizing: border-box;
    background-color: #0c0c0c;
  }

  .control-sidebar {
    width: 320px;
    height: 100%;
    background-color: #1a1a1a;
    border-right: 1px solid #2d2d2d;
    padding: 16px;
    box-sizing: border-box;
    display: flex;
    flex-direction: column;
    gap: 16px;
    overflow-y: auto;
  }

  .control-sidebar h2 {
    margin: 0;
    font-size: 20px;
    color: #e0e0e0;
  }

  .card {
    background-color: #242424;
    border-radius: 8px;
    padding: 12px;
    border: 1px solid #333;
  }

  .card h3 {
    margin: 0 0 10px 0;
    font-size: 16px;
    color: #b0b0b0;
    border-bottom: 1px solid #333;
    padding-bottom: 4px;
  }

  .form-group {
    display: flex;
    flex-direction: column;
    gap: 4px;
    margin-bottom: 10px;
  }

  .form-group label {
    font-size: 12px;
    color: #888;
  }

  .form-group input[type="text"], .form-group select {
    background-color: #1a1a1a;
    border: 1px solid #444;
    color: #fff;
    padding: 6px;
    border-radius: 4px;
  }

  input[type="range"] {
    width: 100%;
  }

  .btn {
    width: 100%;
    padding: 10px;
    border-radius: 6px;
    border: none;
    font-weight: bold;
    font-size: 14px;
    cursor: pointer;
    text-align: center;
  }

  .btn-primary { background-color: #3498db; color: #fff; }
  .btn-success { background-color: #2ecc71; color: #fff; }
  .btn-warning { background-color: #f39c12; color: #fff; }
  .btn-danger { background-color: #e74c3c; color: #fff; }
  .btn-connect { background-color: #7f8c8d; color: #fff; margin-top: 5px; }

  .btn:disabled {
    opacity: 0.4;
    cursor: not-allowed;
  }

  .abort-button {
    font-size: 18px;
    padding: 14px;
    margin-top: auto;
    border: 2px solid #c0392b;
    animation: pulse 2s infinite;
  }

  .status-indicator {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    margin-right: 8px;
  }

  .status-indicator.connected { background-color: #2ecc71; }
  .status-indicator.disconnected { background-color: #e74c3c; }

  .row { display: flex; flex-direction: row; }
  .align-center { align-items: center; }
  .margin-top-xs { margin-top: 6px; }

  .viewport-main {
    flex-grow: 1;
    display: flex;
    flex-direction: column;
    height: 100%;
    box-sizing: border-box;
  }

  .bar-header {
    height: 50px;
    background-color: #141414;
    border-bottom: 1px solid #2d2d2d;
    display: flex;
    align-items: center;
    padding: 0 16px;
  }

  .telemetry-block {
    font-size: 14px;
    color: #aaa;
  }

  .visualizer-container {
    flex-grow: 1;
    padding: 16px;
    box-sizing: border-box;
  }

  .logs-console {
    height: 180px;
    background-color: #111;
    border-top: 1px solid #2a2a2a;
    padding: 12px;
    display: flex;
    flex-direction: column;
  }

  .logs-console h4 {
    margin: 0 0 8px 0;
    font-size: 12px;
    color: #666;
    text-transform: uppercase;
  }

  .logs-feed {
    flex-grow: 1;
    overflow-y: auto;
    font-family: monospace;
    font-size: 12px;
    background-color: #080808;
    border: 1px solid #222;
    border-radius: 4px;
    padding: 8px;
    display: flex;
    flex-direction: column-reverse;
    gap: 4px;
  }

  .log-line {
    border-bottom: 1px dashed #1a1a1a;
    padding-bottom: 2px;
    color: #a2b4c7;
  }

  @keyframes pulse {
    0% { transform: scale(1); }
    50% { transform: scale(0.98); }
    100% { transform: scale(1); }
  }
</style>
