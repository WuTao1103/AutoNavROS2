export interface Point {
  x: number;
  y: number;
}

export interface Pose {
  x: number;
  y: number;
  theta: number; // in radians
}

export interface Twist {
  linear: number; // m/s
  angular: number; // rad/s
}

export interface BatteryState {
  percentage: number;
  voltage: number;
  status: 'Charging' | 'Discharging' | 'Full' | 'Unknown';
}

export enum ConnectionStatus {
  CONNECTED = 'Connected',
  DISCONNECTED = 'Disconnected',
  CONNECTING = 'Connecting',
  ERROR = 'Error',
}

export interface LogMessage {
  id: string;
  timestamp: string;
  level: 'INFO' | 'WARN' | 'ERROR';
  node: string;
  message: string;
}

export interface OccupancyGrid {
  width: number;
  height: number;
  resolution: number; // meters per cell
  origin: Pose;
  data: number[]; // 0-100, -1 for unknown
}

export interface RobotState {
  pose: Pose;
  velocity: Twist;
  battery: BatteryState;
  connection: ConnectionStatus;
  mode: 'IDLE' | 'NAVIGATING' | 'MAPPING' | 'EMERGENCY_STOP';
}