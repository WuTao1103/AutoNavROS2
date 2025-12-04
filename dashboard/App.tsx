import React from 'react';
import { useRosSimulation } from './services/rosSimulation';
import { MapVisualizer } from './components/MapVisualizer';
import { ControlPanel } from './components/ControlPanel';
import { StatusWidget } from './components/StatusWidget';
import { LogConsole } from './components/LogConsole';
import { LayoutDashboard, Settings, Menu } from 'lucide-react';

const App: React.FC = () => {
  const {
    robotState,
    mapData,
    lidarScan,
    path,
    logs,
    targetGoal,
    setCmdVel,
    setGoal,
    toggleEmergencyStop
  } = useRosSimulation();

  return (
    <div className="min-h-screen bg-slate-900 text-slate-200 flex flex-col">
      {/* Header */}
      <header className="bg-slate-800 border-b border-slate-700 px-6 py-4 flex items-center justify-between sticky top-0 z-50">
        <div className="flex items-center gap-3">
          <div className="p-2 bg-cyan-500/10 rounded-lg">
            <LayoutDashboard className="w-6 h-6 text-cyan-400" />
          </div>
          <div>
            <h1 className="text-xl font-bold text-white tracking-tight">ROS2 NavViz</h1>
            <p className="text-xs text-slate-400">Turtlebot3 Simulation • {robotState.connection}</p>
          </div>
        </div>
        <div className="flex items-center gap-4">
            <div className={`px-3 py-1 rounded-full text-xs font-bold border ${
                robotState.mode === 'EMERGENCY_STOP' ? 'bg-red-900/30 border-red-500 text-red-500 animate-pulse' :
                robotState.mode === 'NAVIGATING' ? 'bg-cyan-900/30 border-cyan-500 text-cyan-400' :
                'bg-slate-700/50 border-slate-600 text-slate-400'
            }`}>
                STATUS: {robotState.mode}
            </div>
            <button className="p-2 hover:bg-slate-700 rounded-lg transition-colors">
                <Settings className="w-5 h-5 text-slate-400" />
            </button>
            <button className="p-2 hover:bg-slate-700 rounded-lg transition-colors md:hidden">
                <Menu className="w-5 h-5 text-slate-400" />
            </button>
        </div>
      </header>

      {/* Main Content */}
      <main className="flex-1 p-4 md:p-6 max-w-[1600px] mx-auto w-full grid grid-cols-1 lg:grid-cols-12 gap-6">
        
        {/* Left Column: Map Visualization (Takes up 8/12 columns on large screens) */}
        <div className="lg:col-span-8 flex flex-col gap-6">
           <StatusWidget state={robotState} />
           
           <div className="flex-1 min-h-[500px] bg-slate-800 rounded-xl p-1 border border-slate-700 shadow-xl relative">
              <MapVisualizer 
                mapData={mapData}
                robotPose={robotState.pose}
                lidarScan={lidarScan}
                path={path}
                targetGoal={targetGoal}
                onGoalSet={(p) => setGoal({ ...p, theta: 0 })}
              />
              
              {/* Overlay Info */}
              <div className="absolute bottom-4 right-4 bg-slate-900/90 backdrop-blur border border-slate-700 p-3 rounded-lg text-xs space-y-1">
                 <div className="flex justify-between gap-4">
                    <span className="text-slate-400">Map Frame</span>
                    <span className="font-mono">map</span>
                 </div>
                 <div className="flex justify-between gap-4">
                    <span className="text-slate-400">Resolution</span>
                    <span className="font-mono">0.50m</span>
                 </div>
                 <div className="flex justify-between gap-4">
                    <span className="text-slate-400">Update Rate</span>
                    <span className="font-mono text-green-400">20Hz</span>
                 </div>
              </div>
           </div>
        </div>

        {/* Right Column: Controls & Logs (Takes up 4/12 columns on large screens) */}
        <div className="lg:col-span-4 flex flex-col gap-6 h-full">
            <div className="flex-1 min-h-[300px]">
                <ControlPanel 
                    onTwist={setCmdVel}
                    onStop={() => setCmdVel(0, 0)}
                    onEmergencyStop={toggleEmergencyStop}
                    isEmergencyStop={robotState.mode === 'EMERGENCY_STOP'}
                />
            </div>
            
            <div className="shrink-0">
                <LogConsole logs={logs} />
            </div>

            {/* Quick Stats / Footer */}
            <div className="bg-slate-800 rounded-xl p-4 border border-slate-700 text-xs text-slate-400 flex justify-between">
                <span>Topic: /scan</span>
                <span>Points: {lidarScan.length}</span>
                <span>Max: 10m</span>
            </div>
        </div>

      </main>
    </div>
  );
};

export default App;