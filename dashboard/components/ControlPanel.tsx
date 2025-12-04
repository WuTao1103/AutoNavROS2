import React, { useEffect, useState } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, StopCircle, Power } from 'lucide-react';
import { Twist } from '../types';

interface ControlPanelProps {
  onTwist: (linear: number, angular: number) => void;
  onStop: () => void;
  onEmergencyStop: () => void;
  isEmergencyStop: boolean;
}

export const ControlPanel: React.FC<ControlPanelProps> = ({ 
  onTwist, 
  onStop, 
  onEmergencyStop,
  isEmergencyStop 
}) => {
  const [activeKey, setActiveKey] = useState<string | null>(null);

  const handleKeyDown = (e: KeyboardEvent) => {
    if (isEmergencyStop) return;

    let v = 0;
    let w = 0;
    const speed = 0.5;
    const turn = 1.0;

    switch (e.key) {
      case 'ArrowUp': v = speed; setActiveKey('up'); break;
      case 'ArrowDown': v = -speed; setActiveKey('down'); break;
      case 'ArrowLeft': w = -turn; setActiveKey('left'); break;
      case 'ArrowRight': w = turn; setActiveKey('right'); break;
      case ' ': onStop(); setActiveKey('stop'); return;
      default: return;
    }
    
    // In a real app we might combine forward+turn, but this is simple teleop
    onTwist(v, w);
  };

  const handleKeyUp = () => {
    setActiveKey(null);
    if (!isEmergencyStop) onTwist(0, 0);
  };

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isEmergencyStop, onTwist]);

  const btnClass = (key: string) => 
    `p-4 rounded-lg transition-all duration-150 flex items-center justify-center ${
      activeKey === key 
        ? 'bg-cyan-500 text-white shadow-[0_0_15px_rgba(6,182,212,0.5)]' 
        : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
    }`;

  return (
    <div className="bg-slate-800 rounded-xl p-6 border border-slate-700 shadow-xl h-full flex flex-col">
      <h3 className="text-lg font-semibold text-slate-100 mb-6 flex items-center gap-2">
        <Power className="w-5 h-5 text-cyan-400" />
        Manual Control
      </h3>

      <div className="flex-1 flex flex-col items-center justify-center gap-4">
        <div className="flex justify-center w-full">
            <button 
                className={btnClass('up')} 
                onMouseDown={() => { onTwist(0.5, 0); setActiveKey('up'); }}
                onMouseUp={handleKeyUp}
                onTouchStart={() => { onTwist(0.5, 0); setActiveKey('up'); }}
                onTouchEnd={handleKeyUp}
            >
                <ArrowUp className="w-8 h-8" />
            </button>
        </div>
        <div className="flex justify-center gap-4 w-full">
            <button 
                className={btnClass('left')}
                onMouseDown={() => { onTwist(0, -1); setActiveKey('left'); }}
                onMouseUp={handleKeyUp}
            >
                <ArrowLeft className="w-8 h-8" />
            </button>
            <button 
                className={btnClass('stop')}
                onClick={onStop}
            >
                <StopCircle className="w-8 h-8 text-red-400" />
            </button>
            <button 
                className={btnClass('right')}
                onMouseDown={() => { onTwist(0, 1); setActiveKey('right'); }}
                onMouseUp={handleKeyUp}
            >
                <ArrowRight className="w-8 h-8" />
            </button>
        </div>
        <div className="flex justify-center w-full">
            <button 
                className={btnClass('down')}
                onMouseDown={() => { onTwist(-0.5, 0); setActiveKey('down'); }}
                onMouseUp={handleKeyUp}
            >
                <ArrowDown className="w-8 h-8" />
            </button>
        </div>
      </div>

      <div className="mt-8">
        <button
          onClick={onEmergencyStop}
          className={`w-full py-4 rounded-lg font-bold text-lg tracking-wider uppercase transition-all duration-200 ${
            isEmergencyStop 
              ? 'bg-red-500 text-white animate-pulse shadow-[0_0_20px_rgba(239,68,68,0.6)]' 
              : 'bg-red-900/30 text-red-500 border border-red-900/50 hover:bg-red-900/50'
          }`}
        >
          {isEmergencyStop ? 'RELEASE E-STOP' : 'EMERGENCY STOP'}
        </button>
      </div>
    </div>
  );
};