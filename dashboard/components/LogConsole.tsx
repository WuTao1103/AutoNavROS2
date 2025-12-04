import React, { useEffect, useRef } from 'react';
import { Terminal } from 'lucide-react';
import { LogMessage } from '../types';

interface LogConsoleProps {
  logs: LogMessage[];
}

export const LogConsole: React.FC<LogConsoleProps> = ({ logs }) => {
  const endRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [logs]);

  return (
    <div className="bg-slate-900 rounded-xl border border-slate-700 flex flex-col h-64 shadow-xl overflow-hidden font-mono text-sm">
      <div className="bg-slate-800 px-4 py-2 border-b border-slate-700 flex items-center gap-2">
        <Terminal className="w-4 h-4 text-slate-400" />
        <span className="text-slate-400 font-semibold">/rosout</span>
      </div>
      <div className="flex-1 overflow-y-auto p-4 space-y-1 scrollbar-hide">
        {logs.length === 0 && <span className="text-slate-600 italic">Waiting for nodes...</span>}
        {logs.map((log) => (
          <div key={log.id} className="flex gap-2 hover:bg-slate-800/50 p-0.5 rounded">
            <span className="text-slate-500 shrink-0">[{log.timestamp}]</span>
            <span className={`shrink-0 w-12 font-bold ${
              log.level === 'ERROR' ? 'text-red-500' : 
              log.level === 'WARN' ? 'text-yellow-500' : 'text-cyan-500'
            }`}>
              [{log.level}]
            </span>
            <span className="text-slate-400 shrink-0">[{log.node}]:</span>
            <span className="text-slate-300 break-all">{log.message}</span>
          </div>
        ))}
        <div ref={endRef} />
      </div>
    </div>
  );
};