import React, { useRef, useEffect } from 'react';
import * as d3 from 'd3';
import { OccupancyGrid, Pose, Point } from '../types';
import { MAP_SIZE, MAP_RESOLUTION, LIDAR_MAX_RANGE } from '../constants';

interface MapVisualizerProps {
  mapData: OccupancyGrid | null;
  robotPose: Pose;
  lidarScan: number[];
  path: Pose[];
  targetGoal: Pose | null;
  onGoalSet: (p: Point) => void;
}

export const MapVisualizer: React.FC<MapVisualizerProps> = ({
  mapData,
  robotPose,
  lidarScan,
  path,
  targetGoal,
  onGoalSet,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // Helper to transform world coordinates to pixel coordinates
  // We want the map to fill the canvas while maintaining aspect ratio
  const getTransform = (canvasWidth: number, canvasHeight: number) => {
    const worldWidth = MAP_SIZE * MAP_RESOLUTION;
    const worldHeight = (mapData?.height || MAP_SIZE) * MAP_RESOLUTION;
    
    const scaleX = canvasWidth / worldWidth;
    const scaleY = canvasHeight / worldHeight;
    const scale = Math.min(scaleX, scaleY) * 0.9; // 90% fill for padding

    const offsetX = (canvasWidth - worldWidth * scale) / 2;
    const offsetY = (canvasHeight - worldHeight * scale) / 2;

    return { scale, offsetX, offsetY };
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapData) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const { width, height } = canvas;
    const { scale, offsetX, offsetY } = getTransform(width, height);

    // Clear Canvas
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#0f172a'; // match bg-slate-900
    ctx.fillRect(0, 0, width, height);

    // 1. Draw Map Grid
    // To optimize, usually we'd cache the static map in an offscreen canvas, 
    // but for 40x20 grid, realtime redraw is fine.
    mapData.data.forEach((val, i) => {
      const mx = i % mapData.width;
      const my = Math.floor(i / mapData.width);
      
      const x = offsetX + (mx * MAP_RESOLUTION) * scale;
      const y = offsetY + (my * MAP_RESOLUTION) * scale;
      const size = MAP_RESOLUTION * scale;

      if (val === 100) {
        ctx.fillStyle = '#334155'; // Wall color (slate-700)
        ctx.fillRect(x, y, size + 1, size + 1); // +1 to prevent subpixel gaps
      } else if (val === 0) {
        ctx.fillStyle = '#1e293b'; // Free space (slate-800)
        ctx.fillRect(x, y, size, size);
      }
      // val === -1 (unknown) is transparent/background
    });

    // 2. Draw Grid Lines (Optional, every 1 meter)
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for(let i=0; i<=MAP_SIZE; i+=2) { // Every 1 meter (2 cells)
        const x = offsetX + (i * MAP_RESOLUTION) * scale;
        ctx.moveTo(x, offsetY);
        ctx.lineTo(x, offsetY + ((mapData.height || 0) * MAP_RESOLUTION) * scale);
    }
    // Horizontal lines
    const mapH = mapData.height || MAP_SIZE;
    for(let i=0; i<=mapH; i+=2) {
        const y = offsetY + (i * MAP_RESOLUTION) * scale;
        ctx.moveTo(offsetX, y);
        ctx.lineTo(offsetX + (MAP_SIZE * MAP_RESOLUTION) * scale, y);
    }
    ctx.stroke();

    // 3. Draw Path
    if (path.length > 1) {
      ctx.strokeStyle = '#06b6d4'; // cyan-500
      ctx.lineWidth = 2;
      ctx.beginPath();
      path.forEach((p, i) => {
        const px = offsetX + p.x * scale;
        const py = offsetY + p.y * scale;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    // 4. Draw Goal
    if (targetGoal) {
      const gx = offsetX + targetGoal.x * scale;
      const gy = offsetY + targetGoal.y * scale;
      ctx.fillStyle = '#10b981'; // emerald-500
      ctx.beginPath();
      ctx.arc(gx, gy, 6, 0, 2 * Math.PI);
      ctx.fill();
      // Pulse effect ring
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(gx, gy, 10, 0, 2 * Math.PI);
      ctx.stroke();
    }

    // 5. Draw Robot
    const rx = offsetX + robotPose.x * scale;
    const ry = offsetY + robotPose.y * scale;
    
    // Robot Body
    ctx.fillStyle = '#f59e0b'; // amber-500
    ctx.beginPath();
    ctx.arc(rx, ry, 8, 0, 2 * Math.PI);
    ctx.fill();

    // Robot Heading
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(
      rx + Math.cos(robotPose.theta) * 15,
      ry + Math.sin(robotPose.theta) * 15
    );
    ctx.stroke();

    // 6. Draw LiDAR
    ctx.fillStyle = '#ef4444'; // red-500 points
    lidarScan.forEach((dist, i) => {
      // Calculate angle for this ray
      const angleIncrement = (2 * Math.PI) / lidarScan.length;
      const angle = robotPose.theta + i * angleIncrement;
      
      // If it hit something within max range
      if (dist < LIDAR_MAX_RANGE * 0.99) {
          const lx = rx + (Math.cos(angle) * dist * scale);
          const ly = ry + (Math.sin(angle) * dist * scale);
          
          ctx.beginPath();
          ctx.arc(lx, ly, 2, 0, 2 * Math.PI);
          ctx.fill();
      }
    });

  }, [mapData, robotPose, lidarScan, path, targetGoal]);

  // Handle click for goal setting
  const handleClick = (e: React.MouseEvent) => {
      const canvas = canvasRef.current;
      if (!canvas || !mapData) return;

      const rect = canvas.getBoundingClientRect();
      // Get mouse position relative to the displayed canvas
      const displayX = e.clientX - rect.left;
      const displayY = e.clientY - rect.top;

      // Convert display coordinates to canvas logical coordinates
      // Canvas might be scaled by CSS, so we need to account for that
      const scaleX = canvas.width / rect.width;
      const scaleY = canvas.height / rect.height;
      const canvasX = displayX * scaleX;
      const canvasY = displayY * scaleY;

      // Now use canvas logical dimensions for transform calculation
      const { scale, offsetX, offsetY } = getTransform(canvas.width, canvas.height);

      // Convert canvas coordinates to world coordinates
      const worldX = (canvasX - offsetX) / scale;
      const worldY = (canvasY - offsetY) / scale;

      // Validate bounds
      if (worldX >= 0 && worldX <= MAP_SIZE * MAP_RESOLUTION &&
          worldY >= 0 && worldY <= (mapData.height * MAP_RESOLUTION)) {
              onGoalSet({ x: worldX, y: worldY });
          }
  };

  return (
    <div ref={containerRef} className="w-full h-full relative bg-slate-950 rounded-lg overflow-hidden border border-slate-700 shadow-inner">
        <canvas 
            ref={canvasRef}
            width={800}
            height={600}
            className="w-full h-full cursor-crosshair block"
            onClick={handleClick}
        />
        <div className="absolute top-4 left-4 text-xs text-slate-400 pointer-events-none select-none bg-slate-900/80 p-2 rounded">
            <p>Left Click to Set Goal</p>
            <p className="mt-1 text-cyan-400">Resolution: {MAP_RESOLUTION}m/px</p>
        </div>
    </div>
  );
};