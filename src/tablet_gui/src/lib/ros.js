import ROSLIB from 'roslib';
import { writable } from 'svelte/store';

// Writable stores for ROS status and telemetry
export const rosConnected = writable(false);
export const gantryPosition = writable({ x: 0, y: 0, z: 0 });
export const livePointCloud = writable([]);
export const scanLogs = writable([]);

let rosClient = null;
let posSub = null;
let cloudSub = null;

export function initRos(url = 'ws://localhost:9090') {
  if (typeof window === 'undefined') return null; // Avoid running during build time

  if (rosClient) {
    return rosClient;
  }

  rosClient = new ROSLIB.Ros({ url });

  rosClient.on('connection', () => {
    console.log('Connected to ROSbridge WebSocket!');
    rosConnected.set(true);
    subscribeToTelemetry();
  });

  rosClient.on('error', (error) => {
    console.error('ROSbridge Connection Error:', error);
    rosConnected.set(false);
  });

  rosClient.on('close', () => {
    console.warn('ROSbridge Connection Closed.');
    rosConnected.set(false);
  });

  return rosClient;
}

function subscribeToTelemetry() {
  if (!rosClient) return;

  // Gantry position subscription
  posSub = new ROSLIB.Topic({
    ros: rosClient,
    name: '/current_position',
    messageType: 'geometry_msgs/Point'
  });

  posSub.subscribe((message) => {
    gantryPosition.set({
      x: message.x,
      y: message.y,
      z: message.z
    });
  });

  // Downsampled pointcloud subscription
  cloudSub = new ROSLIB.Topic({
    ros: rosClient,
    name: '/point_cloud/downsampled',
    messageType: 'sensor_msgs/PointCloud2'
  });

  cloudSub.subscribe((message) => {
    // Basic point cloud format translation (header + parsed points list)
    // ThreeCanvas.svelte will parse the raw binary frame for performance.
    livePointCloud.set(message);
  });
}

// Call local gantry E-stop / soft abort
export function publishSoftAbort() {
  if (!rosClient) return;

  const abortTopic = new ROSLIB.Topic({
    ros: rosClient,
    name: '/estop',
    messageType: 'std_msgs/Bool'
  });

  const msg = new ROSLIB.Message({ data: true });
  abortTopic.publish(msg);
  scanLogs.update(logs => ["---> SOFTWARE ABORT COMMAND SENT!", ...logs]);
}

// Invoke Action Server: /scan_body
export function triggerScanBody(startPos, endPos, speed, onFeedback, onResult) {
  if (!rosClient) return null;

  const actionClient = new ROSLIB.ActionClient({
    ros: rosClient,
    serverName: '/scan_body',
    actionName: 'smart_massage_interfaces/action/ScanBody'
  });

  const goal = new ROSLIB.Goal({
    actionClient: actionClient,
    goalMessage: {
      start_position_mm: parseFloat(startPos),
      end_position_mm: parseFloat(endPos),
      speed_mm_s: parseFloat(speed)
    }
  });

  goal.on('feedback', (feedback) => {
    scanLogs.update(logs => [
      `[Feedback] ${feedback.current_phase}: ${feedback.percent_complete}%`,
      ...logs
    ]);
    if (onFeedback) onFeedback(feedback);
  });

  goal.on('result', (result) => {
    scanLogs.update(logs => [
      `[Result] Scan complete (Success: ${result.success}) - ${result.status_message}`,
      ...logs
    ]);
    if (onResult) onResult(result);
  });

  goal.send();
  return goal;
}

// Invoke Action Server: /execute_massage_plan
export function triggerExecuteMassage(points, forceLimit, pattern, onFeedback, onResult) {
  if (!rosClient) return null;

  const actionClient = new ROSLIB.ActionClient({
    ros: rosClient,
    serverName: '/execute_massage_plan',
    actionName: 'smart_massage_interfaces/action/ExecuteMassagePlan'
  });

  const goal = new ROSLIB.Goal({
    actionClient: actionClient,
    goalMessage: {
      selected_points: points,
      force_limit_n: parseFloat(forceLimit),
      massage_pattern: pattern
    }
  });

  goal.on('feedback', (feedback) => {
    scanLogs.update(logs => [
      `[Massage Execution] Waypoint index: ${feedback.current_point_index}, Force: ${feedback.current_force_n.toFixed(1)} N`,
      ...logs
    ]);
    if (onFeedback) onFeedback(feedback);
  });

  goal.on('result', (result) => {
    scanLogs.update(logs => [
      `[Massage Execution Result] Success: ${result.success} - ${result.status_message}`,
      ...logs
    ]);
    if (onResult) onResult(result);
  });

  goal.send();
  return goal;
}
