/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React, { useEffect, useRef, useState, useMemo } from 'react';
import { Play, Pause, RotateCcw, Settings2, BarChart3, Ruler, Layers, ChevronRight, ChevronLeft } from 'lucide-react';
import { 
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, ReferenceLine, ReferenceDot 
} from 'recharts';
import { 
  DEFAULT_DIMENSIONS, DEFAULT_PRB, DEFAULT_MATERIAL, 
  solveLinkage, forwardKinematics, calculateCrankTorque, calculatePotentialEnergy 
} from '../lib/solver';
import { LinkageDimensions, PRBParameters, MaterialProperties } from '../types';
import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';

function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

const LinkageSimulator: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [psi, setPsi] = useState(0);
  const [isPlaying, setIsPlaying] = useState(true);
  const [speed, setSpeed] = useState(1);
  const [showParams, setShowParams] = useState(true);
  
  const [activeTab, setActiveTab] = useState<'params' | 'analysis'>('params');
  const [zoom, setZoom] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [sidebarWidth, setSidebarWidth] = useState(320);
  const [isResizing, setIsResizing] = useState(false);
  const [canvasSize, setCanvasSize] = useState({ width: 0, height: 0 });
  
  const [dims, setDims] = useState<LinkageDimensions>(DEFAULT_DIMENSIONS);
  const [prb, setPrb] = useState<PRBParameters>(DEFAULT_PRB);
  const [material, setMaterial] = useState<MaterialProperties>(DEFAULT_MATERIAL);
  
  const initialTheta = useMemo<[number, number, number]>(() => {
    return solveLinkage(0, dims, prb, [0.1, 0.1, 0.1]);
  }, [dims, prb]);

  const lastThetaRef = useRef<[number, number, number]>([0.1, 0.1, 0.1]);

  // Reset solver seed when dimensions or PRB parameters change
  useEffect(() => {
    lastThetaRef.current = initialTheta;
  }, [initialTheta]);

  // Current solved state with continuity tracking
  const solvedTheta = useMemo(() => {
    // We use the last known solution as the guess for the current psi
    const sol = solveLinkage(psi, dims, prb, lastThetaRef.current);
    lastThetaRef.current = sol;
    return sol;
  }, [psi, dims, prb]);

  // Torque data for plot - Sequential solving ensures branch continuity
  const torqueData = useMemo(() => {
    const data = [];
    const steps = 360; // 1 degree resolution for ultra-smooth curve
    
    // Start from the stable psi=0 solution
    let sequentialGuess: [number, number, number] = [initialTheta[0], initialTheta[1], initialTheta[2]];
    
    for (let i = 0; i <= steps; i++) {
      const p = (i / steps) * 2 * Math.PI;
      const theta = solveLinkage(p, dims, prb, sequentialGuess);
      sequentialGuess = [theta[0], theta[1], theta[2]]; // Use current solution as guess for next angle
      
      const torque = calculateCrankTorque(theta, p, dims, prb, material, initialTheta);
      const energy = calculatePotentialEnergy(theta, dims, prb, material, initialTheta);
      data.push({
        angle: p * 180 / Math.PI,
        torque: parseFloat(torque.toFixed(5)),
        energy: parseFloat(energy.toFixed(5)),
      });
    }
    return data;
  }, [dims, prb, material]);

  const { currentTorque, currentEnergy } = useMemo(() => {
    return {
      currentTorque: calculateCrankTorque(solvedTheta, psi, dims, prb, material, initialTheta),
      currentEnergy: calculatePotentialEnergy(solvedTheta, dims, prb, material, initialTheta)
    };
  }, [solvedTheta, psi, dims, prb, material, initialTheta]);

  // Calculate tight bounding box for the entire cycle
  const cycleBoundingBox = useMemo(() => {
    let minX = 0, maxX = 0, minY = 0, maxY = 0;
    const steps = 36; // Sufficient for bounding box
    let sequentialGuess: [number, number, number] = [0.1, 0.1, 0.1];
    
    for (let i = 0; i <= steps; i++) {
      const p = (i / steps) * 2 * Math.PI;
      const theta = solveLinkage(p, dims, prb, sequentialGuess);
      sequentialGuess = [...theta];
      
      const { Qx, Qy, theta0 } = forwardKinematics(theta, prb.gamma, dims.l);
      const Ax = Qx + dims.Au * Math.cos(theta0) - dims.Av * Math.sin(theta0);
      const Ay = Qy + dims.Au * Math.sin(theta0) + dims.Av * Math.cos(theta0);
      const Bx = dims.Bx;
      const By = dims.By;
      const Cx = Bx + dims.r * Math.cos(p);
      const Cy = By + dims.r * Math.sin(p);

      // Points to consider: O(0,0), B(Bx,By), Q(Qx,Qy), A(Ax,Ay), C(Cx,Cy)
      const points = [
        {x: 0, y: 0}, {x: Bx, y: By}, {x: Qx, y: Qy}, {x: Ax, y: Ay}, {x: Cx, y: Cy}
      ];
      
      points.forEach(pt => {
        minX = Math.min(minX, pt.x);
        maxX = Math.max(maxX, pt.x);
        minY = Math.min(minY, pt.y);
        maxY = Math.max(maxY, pt.y);
      });
    }
    
    return { minX, maxX, minY, maxY, width: maxX - minX, height: maxY - minY };
  }, [dims, prb]);

  useEffect(() => {
    let animationFrame: number;
    const animate = () => {
      if (isPlaying) {
        setPsi(prev => (prev + 0.02 * speed) % (2 * Math.PI));
      }
      animationFrame = requestAnimationFrame(animate);
    };
    animationFrame = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(animationFrame);
  }, [isPlaying, speed]);

  // Responsive canvas sizing with ResizeObserver
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        setCanvasSize({ width, height });
        
        const canvas = canvasRef.current;
        if (canvas) {
          canvas.width = width;
          canvas.height = height;
        }
      }
    });

    resizeObserver.observe(container);
    return () => resizeObserver.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || canvasSize.width === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw background grid for professional look
    ctx.strokeStyle = '#f1f5f9';
    ctx.lineWidth = 1;
    const gridSize = 50;
    for (let x = 0; x < canvas.width; x += gridSize) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += gridSize) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
    }

    ctx.save();
    
    // Center and scale dynamically based on the full cycle bounding box
    const { minX, maxX, minY, maxY, width: worldWidth, height: worldHeight } = cycleBoundingBox;
    
    const padding = 0.1; // 10% padding
    const paddedWidth = worldWidth * (1 + padding * 2);
    const paddedHeight = worldHeight * (1 + padding * 2);
    
    // Scale to fit while maintaining aspect ratio
    const baseScale = Math.min(canvas.width / paddedWidth, canvas.height / paddedHeight);
    const finalScale = baseScale * zoom;
    
    // Center the bounding box in the canvas, then apply zoom and offset
    ctx.translate(
      canvas.width / 2 - (minX + maxX) / 2 * finalScale + offset.x,
      canvas.height / 2 + (minY + maxY) / 2 * finalScale + offset.y
    );
    ctx.scale(finalScale, -finalScale); 

    const { Qx, Qy, theta0 } = forwardKinematics(solvedTheta, prb.gamma, dims.l);
    const Ax = Qx + dims.Au * Math.cos(theta0) - dims.Av * Math.sin(theta0);
    const Ay = Qy + dims.Au * Math.sin(theta0) + dims.Av * Math.cos(theta0);
    const Bx = dims.Bx;
    const By = dims.By;
    const Cx = Bx + dims.r * Math.cos(psi);
    const Cy = By + dims.r * Math.sin(psi);

    // 1. Draw Ground
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1 / finalScale;
    ctx.beginPath();
    ctx.moveTo(-0.02, 0); ctx.lineTo(0.02, 0);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(Bx - 0.01, By); ctx.lineTo(Bx + 0.01, By);
    ctx.stroke();

    // 2. Draw Flexible Beam (PRB 3R segments)
    const [g0, g1, g2, g3] = prb.gamma;
    const [t1, t2, t3] = solvedTheta;
    const l = dims.l;

    const p0 = { x: 0, y: 0 };
    const p1 = { x: g0 * l, y: 0 };
    const p2 = { x: p1.x + g1 * l * Math.cos(t1), y: p1.y + g1 * l * Math.sin(t1) };
    const p3 = { x: p2.x + g2 * l * Math.cos(t1 + t2), y: p2.y + g2 * l * Math.sin(t1 + t2) };
    const p4 = { x: Qx, y: Qy };

    ctx.lineWidth = 4 / finalScale;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    
    // Beam segments
    ctx.strokeStyle = '#1e293b';
    ctx.beginPath(); ctx.moveTo(p0.x, p0.y); ctx.lineTo(p1.x, p1.y); ctx.stroke();
    ctx.strokeStyle = '#2563eb';
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.lineTo(p3.x, p3.y); ctx.lineTo(p4.x, p4.y); ctx.stroke();

    // 3. Draw Coupler (AQ)
    ctx.strokeStyle = '#059669';
    ctx.beginPath(); ctx.moveTo(p4.x, p4.y); ctx.lineTo(Ax, Ay); ctx.stroke();

    // 4. Draw Crank (BA)
    ctx.strokeStyle = '#d97706';
    ctx.beginPath(); ctx.moveTo(Bx, By); ctx.lineTo(Cx, Cy); ctx.stroke();

    // Labels
    ctx.scale(1, -1);
    ctx.font = `bold ${11 / finalScale}px "JetBrains Mono", monospace`;
    ctx.fillStyle = '#475569';
    ctx.fillText('O', -0.005, 0.005);
    ctx.fillText('B', Bx - 0.005, -By + 0.005);
    ctx.fillText('Q', p4.x + 0.002, -p4.y - 0.002);
    ctx.fillText('A', Ax + 0.002, -Ay - 0.002);

    // Dimension labels
    ctx.font = `${10 / finalScale}px "JetBrains Mono", monospace`;
    ctx.fillStyle = '#94a3b8';
    ctx.fillText(`l=${(dims.l * 1000).toFixed(0)}mm`, dims.l / 2 - 0.01, 0.015);
    
    ctx.restore();
  }, [psi, solvedTheta, dims, prb, zoom, offset, canvasSize]);

  const handleZoom = (delta: number) => {
    setZoom(prev => Math.max(0.1, Math.min(10, prev + delta)));
  };

  const resetView = () => {
    setZoom(1);
    setOffset({ x: 0, y: 0 });
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsResizing(true);
    e.preventDefault();
  };

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (!isResizing) return;
      const newWidth = window.innerWidth - e.clientX;
      if (newWidth > 200 && newWidth < 800) {
        setSidebarWidth(newWidth);
      }
    };

    const handleMouseUp = () => {
      setIsResizing(false);
    };

    if (isResizing) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isResizing]);

  return (
    <div className="flex flex-col h-screen bg-[#f8fafc] text-slate-900 overflow-hidden font-mono">
      {/* Header - Technical Style */}
      <header className="h-14 border-b border-slate-200 bg-white px-6 flex items-center justify-between z-20">
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 bg-slate-900 rounded flex items-center justify-center text-white shrink-0">
            <Layers size={18} />
          </div>
          <div className="flex flex-col">
            <h1 className="text-sm font-black tracking-tighter uppercase whitespace-nowrap">Pseudo-Rigid Body 3R Model</h1>
            <span className="text-[9px] text-slate-400 font-bold uppercase tracking-widest">Compliant Linkage Solver</span>
          </div>
          <div className="hidden xl:block ml-4 pl-4 border-l border-slate-200">
            <p className="text-[9px] text-slate-500 leading-tight max-w-lg italic">
              Su, H. (January 7, 2009). "A Pseudorigid-Body 3R Model for Determining Large Deflection of Cantilever Beams Subject to Tip Loads." ASME. J. Mechanisms Robotics. May 2009; 1(2): 021008.
            </p>
          </div>
        </div>
        
        <div className="flex items-center gap-6">
          <div className="flex items-center gap-4 text-[10px] font-bold text-slate-500 uppercase">
            <div className="flex items-center gap-2">
              <div className="w-2 h-2 rounded-full bg-blue-500"></div>
              <span>Beam: {dims.l.toFixed(2)}m</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-2 h-2 rounded-full bg-amber-500"></div>
              <span>Crank: {dims.r.toFixed(2)}m</span>
            </div>
          </div>
          <button 
            onClick={() => setShowParams(!showParams)}
            className={cn(
              "p-2 rounded transition-colors",
              showParams ? "bg-slate-900 text-white" : "bg-slate-100 text-slate-600 hover:bg-slate-200"
            )}
          >
            <Settings2 size={16} />
          </button>
        </div>
      </header>

      {/* Main Content Area */}
      <div className="flex-1 flex overflow-hidden">
        {/* Left: Visualization */}
        <div className="flex-1 flex flex-col min-w-0 overflow-hidden">
          <div className="flex-1 relative bg-white overflow-hidden" ref={containerRef}>
            <canvas ref={canvasRef} className="w-full h-full" />
            <div className="absolute top-4 left-4 bg-white/80 backdrop-blur p-2 border border-slate-200 rounded text-[10px] font-bold text-slate-500 uppercase tracking-tighter">
              Viewport.01 // Real-time Deflection
            </div>
            <div className="absolute bottom-4 right-4 bg-white/80 backdrop-blur p-2 border border-slate-200 rounded text-[10px] font-bold text-blue-600 uppercase tracking-tighter">
              Torque: {currentTorque.toFixed(5)} N·m
            </div>

            {/* Zoom Controls */}
            <div className="absolute top-4 right-4 flex flex-col gap-1">
              <button 
                onClick={() => handleZoom(0.1)}
                className="w-8 h-8 bg-white/80 backdrop-blur border border-slate-200 rounded flex items-center justify-center text-slate-600 hover:bg-white transition-colors shadow-sm"
                title="Zoom In"
              >
                <ChevronRight size={14} className="-rotate-90" />
              </button>
              <button 
                onClick={() => handleZoom(-0.1)}
                className="w-8 h-8 bg-white/80 backdrop-blur border border-slate-200 rounded flex items-center justify-center text-slate-600 hover:bg-white transition-colors shadow-sm"
                title="Zoom Out"
              >
                <ChevronRight size={14} className="rotate-90" />
              </button>
              <button 
                onClick={resetView}
                className="w-8 h-8 bg-white/80 backdrop-blur border border-slate-200 rounded flex items-center justify-center text-slate-600 hover:bg-white transition-colors shadow-sm"
                title="Reset View"
              >
                <RotateCcw size={12} />
              </button>
            </div>
          </div>

          {/* Bottom: Compact Controls */}
          <div className="h-16 bg-white px-8 flex items-center gap-8 border-t border-slate-200">
            <div className="flex items-center gap-3">
              <button 
                onClick={() => setIsPlaying(!isPlaying)}
                className="w-8 h-8 flex items-center justify-center bg-slate-900 text-white rounded hover:bg-slate-800 transition-all shadow-md"
              >
                {isPlaying ? <Pause size={14} fill="currentColor" /> : <Play size={14} className="ml-0.5" fill="currentColor" />}
              </button>
              <button 
                onClick={() => setPsi(0)}
                className="w-8 h-8 flex items-center justify-center bg-slate-100 text-slate-600 rounded hover:bg-slate-200 transition-all"
              >
                <RotateCcw size={14} />
              </button>
            </div>

            <div className="flex-1 flex flex-col gap-1">
              <div className="flex justify-between items-end">
                <span className="text-[8px] font-black text-slate-400 uppercase tracking-widest">Crank_Position</span>
                <span className="text-[10px] font-black text-blue-600 tabular-nums">{(psi * 180 / Math.PI).toFixed(2)}°</span>
              </div>
              <input 
                type="range" min="0" max={2 * Math.PI} step="0.001" value={psi} 
                onChange={(e) => { setPsi(parseFloat(e.target.value)); setIsPlaying(false); }}
                className="w-full h-1 bg-slate-100 rounded-full appearance-none cursor-pointer accent-slate-900"
              />
            </div>

            <div className="w-24 flex flex-col gap-1">
              <span className="text-[8px] font-black text-slate-400 uppercase tracking-widest">Speed</span>
              <input 
                type="range" min="0.1" max="3" step="0.1" value={speed} 
                onChange={(e) => setSpeed(parseFloat(e.target.value))}
                className="w-full h-1 bg-slate-100 rounded-full appearance-none cursor-pointer accent-slate-900"
              />
            </div>
          </div>
        </div>

        {/* Right: Parameters & Analysis Sidebar (Tabbed) */}
        {showParams && (
          <>
            {/* Resize Handle */}
            <div 
              className={cn(
                "w-1 bg-slate-200 hover:bg-blue-400 cursor-col-resize transition-colors z-30",
                isResizing && "bg-blue-500"
              )}
              onMouseDown={handleMouseDown}
            />
            <aside 
              style={{ width: sidebarWidth }}
              className="border-l border-slate-200 bg-[#fcfcfc] flex flex-col overflow-hidden animate-in slide-in-from-right duration-200"
            >
            <div className="flex flex-col h-full">
              <div className="border-b border-slate-200 bg-white">
                <div className="flex p-1">
                  <button 
                    onClick={() => setActiveTab('params')}
                    className={cn(
                      "flex-1 py-2 text-[10px] font-black uppercase tracking-widest rounded transition-all",
                      activeTab === 'params' 
                        ? "bg-slate-900 text-white shadow-sm" 
                        : "text-slate-400 hover:text-slate-600 hover:bg-slate-50"
                    )}
                  >
                    Params
                  </button>
                  <button 
                    onClick={() => setActiveTab('analysis')}
                    className={cn(
                      "flex-1 py-2 text-[10px] font-black uppercase tracking-widest rounded transition-all",
                      activeTab === 'analysis' 
                        ? "bg-slate-900 text-white shadow-sm" 
                        : "text-slate-400 hover:text-slate-600 hover:bg-slate-50"
                    )}
                  >
                    Analysis
                  </button>
                </div>
              </div>
              
              <div className="flex-1 overflow-y-auto p-5 custom-scrollbar">
                {activeTab === 'params' ? (
                  <div className="space-y-8">
                    <section className="space-y-4">
                      <div className="flex items-center justify-between border-b border-slate-100 pb-2">
                        <h4 className="text-[9px] font-black text-slate-400 uppercase tracking-widest">Material_Props</h4>
                        <div className="text-[8px] font-bold text-slate-300">ISO.9001</div>
                      </div>
                      <div className="grid grid-cols-1 gap-4">
                        <ParamInput label="E (GPa)" value={material.E / 1e9} onChange={(v) => setMaterial({...material, E: v * 1e9})} />
                        <ParamInput label="Thickness (mm)" value={material.thickness * 1000} onChange={(v) => setMaterial({...material, thickness: v / 1000})} />
                        <ParamInput label="Width (mm)" value={material.width * 1000} onChange={(v) => setMaterial({...material, width: v / 1000})} />
                      </div>
                    </section>

                    <section className="space-y-4">
                      <div className="flex items-center justify-between border-b border-slate-100 pb-2">
                        <h4 className="text-[9px] font-black text-slate-400 uppercase tracking-widest">PRB_Factors</h4>
                        <div className="text-[8px] font-bold text-slate-300">3R.MODEL</div>
                      </div>
                      <div className="grid grid-cols-2 gap-x-3 gap-y-4">
                        {prb.gamma.map((g, i) => (
                          <ParamInput key={`g-${i}`} label={`γ${i}`} value={g} onChange={(v) => {
                            const newGamma = [...prb.gamma] as [number, number, number, number];
                            newGamma[i] = v;
                            setPrb({...prb, gamma: newGamma});
                          }} />
                        ))}
                        {prb.k.map((k, i) => (
                          <ParamInput key={`k-${i}`} label={`K${i+1}`} value={k} onChange={(v) => {
                            const newK = [...prb.k] as [number, number, number];
                            newK[i] = v;
                            setPrb({...prb, k: newK});
                          }} />
                        ))}
                      </div>
                    </section>

                    <section className="space-y-4">
                      <div className="flex items-center justify-between border-b border-slate-100 pb-2">
                        <h4 className="text-[9px] font-black text-slate-400 uppercase tracking-widest">Geometry</h4>
                        <div className="text-[8px] font-bold text-slate-300">CAD.REF</div>
                      </div>
                      <div className="grid grid-cols-2 gap-3">
                        <ParamInput label="Beam L (m)" value={dims.l} onChange={(v) => setDims({...dims, l: v})} />
                        <ParamInput label="Crank R (m)" value={dims.r} onChange={(v) => setDims({...dims, r: v})} />
                        <ParamInput label="Attach u" value={dims.Au} onChange={(v) => setDims({...dims, Au: v})} />
                        <ParamInput label="Attach v" value={dims.Av} onChange={(v) => setDims({...dims, Av: v})} />
                        <ParamInput label="Ground X" value={dims.Bx} onChange={(v) => setDims({...dims, Bx: v})} />
                        <ParamInput label="Ground Y" value={dims.By} onChange={(v) => setDims({...dims, By: v})} />
                      </div>
                    </section>

                    <div className="pt-4">
                      <button 
                        onClick={() => { setDims(DEFAULT_DIMENSIONS); setPrb(DEFAULT_PRB); setMaterial(DEFAULT_MATERIAL); }}
                        className="w-full py-2 text-[10px] font-black uppercase tracking-widest text-slate-500 border border-slate-200 rounded hover:bg-slate-50 transition-colors"
                      >
                        Reset_Defaults
                      </button>
                    </div>
                  </div>
                ) : (
                  <div className="h-full flex flex-col">
                    <div className="mb-6">
                      <span className="text-[10px] font-black text-slate-400 uppercase tracking-widest flex items-center gap-2 mb-4">
                        <BarChart3 size={12} /> Analysis.Torque_Curve
                      </span>
                      <div className="h-[280px] w-full">
                        <ResponsiveContainer width="100%" height="100%">
                          <LineChart data={torqueData} margin={{ top: 10, right: 10, left: -20, bottom: 0 }}>
                            <CartesianGrid strokeDasharray="2 2" vertical={false} stroke="#f1f5f9" />
                            <XAxis 
                              dataKey="angle" 
                              type="number" 
                              domain={[0, 360]} 
                              hide 
                            />
                            <YAxis 
                              yAxisId="left"
                              tick={{ fontSize: 8, fontFamily: 'monospace' }} 
                              label={{ value: 'Torque (N·m)', angle: -90, position: 'insideLeft', fontSize: 8, offset: 10 }}
                            />
                            <YAxis 
                              yAxisId="right"
                              orientation="right"
                              tick={{ fontSize: 8, fontFamily: 'monospace' }} 
                              label={{ value: 'Energy (J)', angle: 90, position: 'insideRight', fontSize: 8, offset: 10 }}
                            />
                            <Tooltip 
                              contentStyle={{ fontSize: '10px', fontFamily: 'monospace', borderRadius: '4px', border: '1px solid #e2e8f0' }}
                              labelFormatter={(val) => `Angle: ${Number(val).toFixed(1)}°`}
                            />
                            <Line 
                              yAxisId="left"
                              type="monotone" 
                              dataKey="torque" 
                              stroke="#2563eb" 
                              strokeWidth={2} 
                              dot={false} 
                              isAnimationActive={false} 
                              name="Torque"
                            />
                            <Line 
                              yAxisId="right"
                              type="monotone" 
                              dataKey="energy" 
                              stroke="#059669" 
                              strokeWidth={2} 
                              dot={false} 
                              isAnimationActive={false} 
                              name="Energy"
                            />
                            {/* Current Position Indicator */}
                            <ReferenceLine 
                              yAxisId="left"
                              x={psi * 180 / Math.PI} 
                              stroke="#ef4444" 
                              strokeWidth={2} 
                              strokeDasharray="3 3"
                            />
                            <ReferenceDot 
                              yAxisId="left"
                              x={psi * 180 / Math.PI} 
                              y={currentTorque} 
                              r={4} 
                              fill="#ef4444" 
                              stroke="#fff" 
                              strokeWidth={2} 
                            />
                            <ReferenceDot 
                              yAxisId="right"
                              x={psi * 180 / Math.PI} 
                              y={currentEnergy} 
                              r={4} 
                              fill="#059669" 
                              stroke="#fff" 
                              strokeWidth={2} 
                            />
                          </LineChart>
                        </ResponsiveContainer>
                      </div>
                    </div>
                    
                    <div className="space-y-4">
                      <div className="grid grid-cols-1 gap-2">
                        <div className="p-4 bg-slate-900 rounded-lg text-white shadow-xl">
                          <div className="text-[9px] font-black text-slate-500 uppercase tracking-widest mb-1">Live.Output.Torque</div>
                          <div className="text-xl font-black tabular-nums tracking-tighter">
                            {currentTorque.toFixed(5)} <span className="text-[10px] font-bold text-slate-500">N·m</span>
                          </div>
                        </div>
                        <div className="p-4 bg-emerald-900 rounded-lg text-white shadow-xl">
                          <div className="text-[9px] font-black text-emerald-500 uppercase tracking-widest mb-1">Live.Potential.Energy</div>
                          <div className="text-xl font-black tabular-nums tracking-tighter">
                            {currentEnergy.toFixed(5)} <span className="text-[10px] font-bold text-emerald-500">J</span>
                          </div>
                        </div>
                      </div>

                      <div className="grid grid-cols-2 gap-2">
                        <div className="p-3 bg-white border border-slate-200 rounded">
                          <div className="text-[8px] font-black text-slate-400 uppercase mb-1">Max.Torque</div>
                          <div className="text-xs font-bold tabular-nums">
                            {Math.max(...torqueData.map(d => d.torque)).toFixed(4)}
                          </div>
                        </div>
                        <div className="p-3 bg-white border border-slate-200 rounded">
                          <div className="text-[8px] font-black text-slate-400 uppercase mb-1">Min.Torque</div>
                          <div className="text-xs font-bold tabular-nums">
                            {Math.min(...torqueData.map(d => d.torque)).toFixed(4)}
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </aside>
        </>
      )}
    </div>

      {/* Footer - Status Bar */}
      <footer className="h-8 bg-slate-900 text-slate-400 px-6 flex items-center justify-between text-[9px] font-bold uppercase tracking-widest">
        <div className="flex gap-6">
          <span className="flex items-center gap-2">
            <div className="w-1.5 h-1.5 rounded-full bg-emerald-500"></div>
            Solver.Status: Stable
          </span>
          <span>Mode: Static_Equilibrium</span>
        </div>
        <div className="flex gap-4">
          <span>User: {process.env.USER_EMAIL || 'haijun.su@gmail.com'}</span>
          <span>TS: {new Date().toISOString().split('T')[1].split('.')[0]}</span>
        </div>
      </footer>
    </div>
  );
};

const ParamInput: React.FC<{ label: string, value: number, onChange: (v: number) => void }> = ({ label, value, onChange }) => (
  <div className="flex flex-col gap-1">
    <label className="text-[8px] font-black text-slate-400 uppercase tracking-tighter">{label}</label>
    <input 
      type="number" 
      value={value} 
      step="any"
      onChange={(e) => onChange(parseFloat(e.target.value))}
      className="w-full bg-white border border-slate-200 rounded px-2 py-1.5 text-[10px] font-bold focus:border-slate-900 outline-none transition-colors"
    />
  </div>
);

export default LinkageSimulator;
