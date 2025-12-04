import { useState, useEffect, useRef, useCallback } from 'react';
import { 
  RobotState, 
  Pose, 
  Twist, 
  OccupancyGrid, 
  ConnectionStatus, 
  LogMessage 
} from '../types';
import { 
  MAP_SIZE, 
  MAP_RESOLUTION, 
  UPDATE_RATE_MS, 
  LIDAR_RAYS, 
  MOCK_MAP_LAYOUT,
  LIDAR_MAX_RANGE
} from '../constants';

// Helper to generate UUIDs for logs
const generateId = () => Math.random().toString(36).substr(2, 9);

export const useRosSimulation = () => {
  // --- State ---
  // Initial pose set to center of the 20m x 9m map (x=10, y=4.5)
  const [robotState, setRobotState] = useState<RobotState>({
    pose: { x: 10, y: 4.5, theta: 0 },
    velocity: { linear: 0, angular: 0 },
    battery: { percentage: 98, voltage: 12.4, status: 'Discharging' },
    connection: ConnectionStatus.CONNECTED,
    mode: 'IDLE',
  });

  const [mapData, setMapData] = useState<OccupancyGrid | null>(null);
  const [lidarScan, setLidarScan] = useState<number[]>([]);
  const [path, setPath] = useState<Pose[]>([]);
  const [logs, setLogs] = useState<LogMessage[]>([]);
  const [targetGoal, setTargetGoal] = useState<Pose | null>(null);

  // --- Refs for mutable physics state without re-rendering loop ---
  const poseRef = useRef<Pose>({ x: 10, y: 4.5, theta: 0 });
  const velocityRef = useRef<Twist>({ linear: 0, angular: 0 });
  const mapRef = useRef<number[]>([]);
  const isSimulatingRef = useRef(true);

  // --- Map Initialization ---
  useEffect(() => {
    // Convert ASCII layout to linear OccupancyGrid array
    const width = MAP_SIZE;
    const height = MOCK_MAP_LAYOUT.length > MAP_SIZE ? MAP_SIZE : MOCK_MAP_LAYOUT.length;
    const data = new Array(width * height).fill(-1);

    MOCK_MAP_LAYOUT.forEach((row, y) => {
      for (let x = 0; x < row.length; x++) {
        if (x < width) {
           // 100 = occupied, 0 = free
           const val = row[x] === '1' ? 100 : 0;
           data[y * width + x] = val;
        }
      }
    });

    mapRef.current = data;
    setMapData({
      width,
      height,
      resolution: MAP_RESOLUTION,
      origin: { x: 0, y: 0, theta: 0 },
      data
    });

    addLog('INFO', '/map_server', 'Occupancy grid map received');
    addLog('INFO', '/amcl', 'Initial pose estimate converged');
  }, []);

  // --- Logging Helper ---
  const addLog = (level: 'INFO' | 'WARN' | 'ERROR', node: string, message: string) => {
    setLogs(prev => {
      const newLog = {
        id: generateId(),
        timestamp: new Date().toLocaleTimeString(),
        level,
        node,
        message
      };
      return [newLog, ...prev].slice(0, 50); // Keep last 50 logs
    });
  };

  // --- Physics & Sensor Loop ---
  useEffect(() => {
    const interval = setInterval(() => {
      if (!isSimulatingRef.current || robotState.mode === 'EMERGENCY_STOP') return;

      // 1. Update Pose based on Velocity (Simple Kinematics)
      const dt = UPDATE_RATE_MS / 1000;
      const v = velocityRef.current.linear;
      const w = velocityRef.current.angular;
      const theta = poseRef.current.theta;

      // Calculate potential new position
      const dx = v * Math.cos(theta) * dt;
      const dy = v * Math.sin(theta) * dt;
      let newX = poseRef.current.x + dx;
      let newY = poseRef.current.y + dy;
      let newTheta = theta + (w * dt);

      // Normalize theta
      newTheta = newTheta % (2 * Math.PI);
      if (newTheta < 0) newTheta += 2 * Math.PI;

      // --- Collision Detection ---
      const isWalkable = (x: number, y: number) => {
        const mx = Math.floor(x / MAP_RESOLUTION);
        const my = Math.floor(y / MAP_RESOLUTION);
        
        // Bounds check
        if (mx < 0 || mx >= MAP_SIZE || my < 0 || my >= MOCK_MAP_LAYOUT.length) {
          return false;
        }

        // Check if map data is loaded
        if (mapRef.current.length === 0) return true; // Assume safe if no map yet

        // Occupancy check (0 = free, 100 = occupied)
        const idx = my * MAP_SIZE + mx;
        return mapRef.current[idx] < 50; 
      };

      // Try moving along X axis first
      let finalX = poseRef.current.x;
      let finalY = poseRef.current.y;

      if (isWalkable(newX, poseRef.current.y)) {
        finalX = newX;
      } 

      // Try moving along Y axis (using potentially new X if we allow diagonal sliding)
      // We check (finalX, newY) to ensure we don't corner-clip into a wall if both changed
      if (isWalkable(finalX, newY)) {
        finalY = newY;
      } else if (isWalkable(poseRef.current.x, newY)) {
         // If diagonal move failed, but vertical move is valid (sliding against vertical wall)
         // we might want to prioritize Y over X depending on angle, 
         // but simple "keep X if valid, then keep Y if valid" works well for sliding.
         // In this specific branch: (finalX, newY) failed. 
         // If finalX changed, maybe we should keep finalX and discard Y?
         // Or discard finalX and keep Y?
         // For simple sliding, we just don't update Y if the combined move fails.
      }

      // Update pose
      poseRef.current = { x: finalX, y: finalY, theta: newTheta };

      // 2. Simulate LiDAR
      const scan = simulateLidar(poseRef.current, mapRef.current);
      setLidarScan(scan);

      // 3. Update Path (Breadcrumbs for visualization)
      // Only add point if moved significantly
      setPath(prev => {
        const last = prev[prev.length - 1];
        if (!last || Math.hypot(last.x - finalX, last.y - finalY) > 0.5) {
          return [...prev, { ...poseRef.current }].slice(-200); // keep last 200 points
        }
        return prev;
      });

      // 4. Navigation Logic (Mock MoveBase)
      if (targetGoal && robotState.mode === 'NAVIGATING') {
        const dx = targetGoal.x - finalX;
        const dy = targetGoal.y - finalY;
        const dist = Math.hypot(dx, dy);
        
        if (dist < 0.5) {
          addLog('INFO', '/nav2', 'Goal reached successfully');
          setTargetGoal(null);
          setRobotState(s => ({ ...s, mode: 'IDLE' }));
          setCmdVel(0, 0);
        } else {
          // Simple P-controller for demo
          const angleToGoal = Math.atan2(dy, dx);
          let angleDiff = angleToGoal - newTheta;
          // Normalize angle diff
          while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
          while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

          const kp_linear = 1.0;
          const kp_angular = 2.0;
          
          let cmd_v = dist * kp_linear;
          let cmd_w = angleDiff * kp_angular;

          // Cap speeds
          cmd_v = Math.min(cmd_v, 1.0);
          cmd_w = Math.max(Math.min(cmd_w, 1.5), -1.5);
          
          // Don't move forward if not facing roughly the right way
          if (Math.abs(angleDiff) > 0.5) cmd_v = 0;

          setCmdVel(cmd_v, cmd_w);
        }
      }

      // Update React State for UI
      setRobotState(prev => ({
        ...prev,
        pose: poseRef.current,
        velocity: velocityRef.current
      }));

    }, UPDATE_RATE_MS);

    return () => clearInterval(interval);
  }, [robotState.mode, targetGoal]);

  // --- Raycasting for Lidar Simulation ---
  const simulateLidar = (pose: Pose, map: number[]) => {
    const ranges: number[] = [];
    const angleIncrement = (2 * Math.PI) / LIDAR_RAYS;
    
    for (let i = 0; i < LIDAR_RAYS; i++) {
      const rayAngle = pose.theta + (i * angleIncrement);
      let dist = 0;
      let hit = false;
      
      const stepSize = 0.1; // 10cm steps
      const cx = Math.cos(rayAngle);
      const cy = Math.sin(rayAngle);

      while (dist < LIDAR_MAX_RANGE && !hit) {
        dist += stepSize;
        const px = pose.x + cx * dist;
        const py = pose.y + cy * dist;

        // Map coordinates
        const mx = Math.floor(px / MAP_RESOLUTION);
        const my = Math.floor(py / MAP_RESOLUTION);

        if (mx < 0 || mx >= MAP_SIZE || my < 0 || my >= MOCK_MAP_LAYOUT.length) {
          hit = true; // Out of bounds is a hit
        } else {
           const idx = my * MAP_SIZE + mx;
           if (map[idx] > 50) { // Wall
             hit = true;
           }
        }
      }
      ranges.push(dist);
    }
    return ranges;
  };

  // --- Public Control Methods ---
  const setCmdVel = useCallback((linear: number, angular: number) => {
    velocityRef.current = { linear, angular };
    // Also update state immediately for UI responsiveness
    setRobotState(prev => ({ 
      ...prev, 
      velocity: { linear, angular },
      mode: prev.mode === 'EMERGENCY_STOP' ? 'EMERGENCY_STOP' : (linear !== 0 || angular !== 0 ? 'NAVIGATING' : 'IDLE')
    }));
  }, []);

  const setGoal = useCallback((pose: Pose) => {
    setTargetGoal(pose);
    setRobotState(prev => ({ ...prev, mode: 'NAVIGATING' }));
    addLog('INFO', '/move_base', `Received new goal: x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}`);
  }, []);

  const toggleEmergencyStop = useCallback(() => {
    setRobotState(prev => {
      const newMode = prev.mode === 'EMERGENCY_STOP' ? 'IDLE' : 'EMERGENCY_STOP';
      if (newMode === 'EMERGENCY_STOP') {
        velocityRef.current = { linear: 0, angular: 0 };
        setTargetGoal(null);
        addLog('ERROR', '/safety_controller', 'EMERGENCY STOP TRIGGERED');
      } else {
        addLog('INFO', '/safety_controller', 'Emergency stop released');
      }
      return { ...prev, mode: newMode };
    });
  }, []);

  return {
    robotState,
    mapData,
    lidarScan,
    path,
    logs,
    targetGoal,
    setCmdVel,
    setGoal,
    toggleEmergencyStop,
  };
};