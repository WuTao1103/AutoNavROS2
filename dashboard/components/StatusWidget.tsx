import React from 'react';
import { Battery, Wifi, Activity, MapPin, Gauge } from 'lucide-react';
import { RobotState } from '../types';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer } from 'recharts';

interface StatusWidgetProps {
  state: RobotState;
}

// Simple mock data for the velocity chart
const data = Array.from({ length: 20 }, (_, i) => ({
  time: i,
  vel: Math.random() * 0.2
}));

export const StatusWidget: React.FC<StatusWidgetProps> = ({ state }) => {
  const getBatteryColor = (level: number) => {
    if (level > 60) return 'text-green-400';
    if (level > 20) return 'text-yellow-400';
    return 'text-red-400';
  };

  return (
    <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
      {/* Battery Status */}
      <div className="bg-slate-800 p-4 rounded-xl border border-slate-700 shadow-md">
        <div className="flex items-center justify-between mb-2">
            <span className="text-slate-400 text-sm font-medium">Battery</span>
            <Battery className={`w-5 h-5 ${getBatteryColor(state.battery.percentage)}`} />
        </div>
        <div className="flex items-end gap-2">
            <span className="text-2xl font-bold text-slate-100">{state.battery.percentage}%</span>
            <span className="text-xs text-slate-500 mb-1">{state.battery.voltage}V</span>
        </div>
        <div className="w-full bg-slate-700 h-1.5 mt-3 rounded-full overflow-hidden">
            <div 
                className={`h-full rounded-full transition-all duration-500 ${state.battery.percentage > 20 ? 'bg-green-500' : 'bg-red-500'}`} 
                style={{ width: `${state.battery.percentage}%` }}
            />
        </div>
      </div>

      {/* Connectivity */}
      <div className="bg-slate-800 p-4 rounded-xl border border-slate-700 shadow-md">
        <div className="flex items-center justify-between mb-2">
            <span className="text-slate-400 text-sm font-medium">Connection</span>
            <Wifi className="w-5 h-5 text-cyan-400" />
        </div>
        <div className="text-lg font-semibold text-slate-100">{state.connection}</div>
        <div className="text-xs text-slate-500 mt-1">Latency: 12ms</div>
      </div>

      {/* Position */}
      <div className="bg-slate-800 p-4 rounded-xl border border-slate-700 shadow-md">
        <div className="flex items-center justify-between mb-2">
            <span className="text-slate-400 text-sm font-medium">Pose</span>
            <MapPin className="w-5 h-5 text-purple-400" />
        </div>
        <div className="grid grid-cols-3 gap-1 text-xs">
            <div className="bg-slate-700/50 p-1 rounded text-center">
                <span className="text-slate-500 block">X</span>
                <span className="text-slate-200">{state.pose.x.toFixed(1)}</span>
            </div>
            <div className="bg-slate-700/50 p-1 rounded text-center">
                <span className="text-slate-500 block">Y</span>
                <span className="text-slate-200">{state.pose.y.toFixed(1)}</span>
            </div>
             <div className="bg-slate-700/50 p-1 rounded text-center">
                <span className="text-slate-500 block">Θ</span>
                <span className="text-slate-200">{state.pose.theta.toFixed(1)}</span>
            </div>
        </div>
      </div>

      {/* Velocity / Status */}
      <div className="bg-slate-800 p-4 rounded-xl border border-slate-700 shadow-md">
        <div className="flex items-center justify-between mb-2">
            <span className="text-slate-400 text-sm font-medium">Velocity</span>
            <Gauge className="w-5 h-5 text-orange-400" />
        </div>
        <div className="flex justify-between items-end">
             <div>
                <span className="text-2xl font-bold text-slate-100">{state.velocity.linear.toFixed(2)}</span>
                <span className="text-xs text-slate-500 ml-1">m/s</span>
             </div>
             <div className="text-right">
                <span className="block text-xs text-slate-400">{state.mode}</span>
             </div>
        </div>
      </div>
    </div>
  );
};