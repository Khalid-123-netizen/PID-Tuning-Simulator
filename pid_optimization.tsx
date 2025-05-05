import { useState, useEffect } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

export default function PIDOptimization() {
  const [kp, setKp] = useState(5);
  const [ki, setKi] = useState(1);
  const [kd, setKd] = useState(2);
  const [timeRange, setTimeRange] = useState(10);
  const [stepSize, setStepSize] = useState(0.01);
  const [windDisturbance, setWindDisturbance] = useState(0.5);
  const [data, setData] = useState([]);
  const [metrics, setMetrics] = useState({ riseTime: 0, overshoot: 0, settlingTime: 0 });
  const [bestGains, setBestGains] = useState({ kp: 5, ki: 1, kd: 2, score: 0 });
  const [autotuneStatus, setAutotuneStatus] = useState('');

  // Simulation parameters
  const m = 1.0; // mass in kg
  const g = 9.81; // gravity
  const h_desired = 10; // desired altitude in meters

  // Run simulation with current PID values
  useEffect(() => {
    runSimulation(kp, ki, kd, timeRange, stepSize, windDisturbance);
  }, [kp, ki, kd, timeRange, stepSize, windDisturbance]);

  function runSimulation(kp, ki, kd, timeLength, dt, windMagnitude) {
    // Initial conditions
    let h = 0; // height
    let v = 0; // velocity
    let integral_error = 0;
    let prev_error = 0;
    
    const simData = [];
    const timePoints = Math.floor(timeLength / dt);
    
    for (let i = 0; i < timePoints; i++) {
      const t = i * dt;
      
      // Calculate error
      const error = h_desired - h;
      
      // Integrate error
      integral_error += error * dt;
      
      // Calculate derivative of error
      const derivative_error = (error - prev_error) / dt;
      prev_error = error;
      
      // PID control law
      const thrust = m * g + kp * error + ki * integral_error + kd * derivative_error;
      
      // Add wind disturbance (random)
      const wind = windMagnitude * (Math.random() * 2 - 1);
      
      // Update dynamics (simple Euler integration)
      const a = (thrust / m) - g + wind;
      v += a * dt;
      h += v * dt;
      
      // Store data
      simData.push({ 
        time: t, 
        altitude: h, 
        setpoint: h_desired, 
        thrust: thrust,
        error: error
      });
    }
    
    // Calculate performance metrics
    const settlingThreshold = 0.02; // 2% criterion
    let riseTime = timeLength;
    let overshoot = 0;
    let settlingTime = timeLength;
    
    // Find rise time (time to reach 90% of setpoint)
    for (let i = 0; i < simData.length; i++) {
      if (simData[i].altitude >= 0.9 * h_desired) {
        riseTime = simData[i].time;
        break;
      }
    }
    
    // Find maximum overshoot
    for (let i = 0; i < simData.length; i++) {
      const overshootAmount = (simData[i].altitude - h_desired) / h_desired * 100;
      overshoot = Math.max(overshoot, overshootAmount);
    }
    
    // Find settling time
    let settled = false;
    for (let i = 0; i < simData.length; i++) {
      const relativeError = Math.abs((simData[i].altitude - h_desired) / h_desired);
      if (relativeError <= settlingThreshold) {
        if (!settled) {
          settled = true;
          settlingTime = simData[i].time;
        }
      } else if (settled) {
        // If we exceed the threshold after settling, we need to reset
        settled = false;
      }
    }
    
    // Update state
    setData(simData);
    setMetrics({ riseTime, overshoot, settlingTime });
  }
  
  // Calculate a score for the current configuration (lower is better)
  const calculateScore = () => {
    const riseTimeWeight = 1.0;
    const overshootWeight = 0.5;
    const settlingTimeWeight = 1.5;
    
    return (
      riseTimeWeight * metrics.riseTime + 
      overshootWeight * metrics.overshoot + 
      settlingTimeWeight * metrics.settlingTime
    );
  };

  // Auto-tune the PID gains
  const autotune = () => {
    setAutotuneStatus('Tuning in progress...');
    
    // Starting ranges for search
    const kpRange = { min: 1, max: 20, step: 1 };
    const kiRange = { min: 0, max: 5, step: 0.5 };
    const kdRange = { min: 0, max: 10, step: 0.5 };
    
    let bestScore = Number.MAX_VALUE;
    let bestConfig = { kp: 5, ki: 1, kd: 2 };
    
    // We'll use a grid search for simplicity
    // For a real application, more advanced optimization methods would be better
    setTimeout(() => {
      const totalSteps = 
        Math.ceil((kpRange.max - kpRange.min) / kpRange.step) * 
        Math.ceil((kiRange.max - kiRange.min) / kiRange.step) * 
        Math.ceil((kdRange.max - kdRange.min) / kdRange.step);
      
      let stepsCompleted = 0;
      
      // Due to browser limitations, we'll simulate a subset of the search
      // In a real application, this would be a complete grid search
      const samples = 100; // Number of random samples to take
      
      for (let i = 0; i < samples; i++) {
        // Generate random values within ranges
        const kpTest = kpRange.min + Math.random() * (kpRange.max - kpRange.min);
        const kiTest = kiRange.min + Math.random() * (kiRange.max - kiRange.min);
        const kdTest = kdRange.min + Math.random() * (kdRange.max - kdRange.min);
        
        // Run simulation with these values
        let h = 0; // height
        let v = 0; // velocity
        let integral_error = 0;
        let prev_error = 0;
        const dt = 0.01;
        const timeLength = 10;
        const timePoints = Math.floor(timeLength / dt);
        let riseTime = timeLength;
        let maxOvershoot = 0;
        let settlingTime = timeLength;
        let settled = false;
        
        for (let t = 0; t < timePoints; t++) {
          const time = t * dt;
          
          // Calculate error
          const error = h_desired - h;
          
          // Integrate error
          integral_error += error * dt;
          
          // Calculate derivative of error
          const derivative_error = (error - prev_error) / dt;
          prev_error = error;
          
          // PID control law
          const thrust = m * g + kpTest * error + kiTest * integral_error + kdTest * derivative_error;
          
          // Add wind disturbance (random)
          const wind = windDisturbance * (Math.random() * 2 - 1);
          
          // Update dynamics (simple Euler integration)
          const a = (thrust / m) - g + wind;
          v += a * dt;
          h += v * dt;
          
          // Check rise time (time to reach 90% of setpoint)
          if (h >= 0.9 * h_desired && riseTime === timeLength) {
            riseTime = time;
          }
          
          // Check maximum overshoot
          const overshootAmount = (h - h_desired) / h_desired * 100;
          if (overshootAmount > maxOvershoot) {
            maxOvershoot = overshootAmount;
          }
          
          // Check settling time (2% criterion)
          const relativeError = Math.abs((h - h_desired) / h_desired);
          if (relativeError <= 0.02) {
            if (!settled) {
              settled = true;
              settlingTime = time;
            }
          } else if (settled) {
            // If we exceed the threshold after settling, we need to reset
            settled = false;
            settlingTime = timeLength;
          }
        }
        
        // Calculate score
        const score = 
          1.0 * riseTime + 
          0.5 * maxOvershoot + 
          1.5 * settlingTime;
        
        // Update best if better
        if (score < bestScore) {
          bestScore = score;
          bestConfig = { kp: kpTest, ki: kiTest, kd: kdTest };
        }
        
        stepsCompleted++;
      }
      
      // Update the state with the best configuration
      setBestGains({ ...bestConfig, score: bestScore });
      setAutotuneStatus(`Tuning complete! Found best gains with score: ${bestScore.toFixed(2)}`);
    }, 100);
  };

  // Apply the best gains
  const applyBestGains = () => {
    setKp(bestGains.kp);
    setKi(bestGains.ki);
    setKd(bestGains.kd);
  };

  return (
    <div className="p-4">
      <h1 className="text-2xl font-bold mb-4">Drone Altitude Control PID Tuning</h1>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-4">
        <div className="p-4 border rounded">
          <h2 className="text-lg font-semibold mb-2">PID Controller Parameters</h2>
          <div className="grid grid-cols-2 gap-2">
            <div>
              <label className="block text-sm font-medium">Kp: {kp.toFixed(2)}</label>
              <input 
                type="range" 
                min="0" 
                max="20" 
                step="0.1" 
                value={kp} 
                onChange={(e) => setKp(parseFloat(e.target.value))}
                className="w-full"
              />
            </div>
            <div>
              <label className="block text-sm font-medium">Ki: {ki.toFixed(2)}</label>
              <input 
                type="range" 
                min="0" 
                max="5" 
                step="0.1" 
                value={ki} 
                onChange={(e) => setKi(parseFloat(e.target.value))}
                className="w-full"
              />
            </div>
            <div>
              <label className="block text-sm font-medium">Kd: {kd.toFixed(2)}</label>
              <input 
                type="range" 
                min="0" 
                max="10" 
                step="0.1" 
                value={kd} 
                onChange={(e) => setKd(parseFloat(e.target.value))}
                className="w-full"
              />
            </div>
            <div>
              <label className="block text-sm font-medium">Wind Disturbance: {windDisturbance.toFixed(2)}</label>
              <input 
                type="range" 
                min="0" 
                max="2" 
                step="0.1" 
                value={windDisturbance} 
                onChange={(e) => setWindDisturbance(parseFloat(e.target.value))}
                className="w-full"
              />
            </div>
          </div>
          
          <div className="mt-4 grid grid-cols-2 gap-2">
            <button 
              onClick={autotune}
              className="bg-blue-500 hover:bg-blue-600 text-white py-2 px-4 rounded"
            >
              Auto-Tune PID
            </button>
            <button 
              onClick={applyBestGains}
              className="bg-green-500 hover:bg-green-600 text-white py-2 px-4 rounded"
            >
              Apply Best Gains
            </button>
          </div>
          
          {autotuneStatus && (
            <div className="mt-2 text-sm">{autotuneStatus}</div>
          )}
          
          <div className="mt-4">
            <h3 className="font-medium">Best Gains Found:</h3>
            <p>Kp: {bestGains.kp.toFixed(2)}, Ki: {bestGains.ki.toFixed(2)}, Kd: {bestGains.kd.toFixed(2)}</p>
            <p>Score: {bestGains.score.toFixed(2)}</p>
          </div>
        </div>
        
        <div className="p-4 border rounded">
          <h2 className="text-lg font-semibold mb-2">Performance Metrics</h2>
          <div className="grid grid-cols-3 gap-4">
            <div className="text-center p-2 bg-gray-100 rounded">
              <p className="text-sm font-medium">Rise Time</p>
              <p className="text-lg">{metrics.riseTime.toFixed(2)} s</p>
            </div>
            <div className="text-center p-2 bg-gray-100 rounded">
              <p className="text-sm font-medium">Overshoot</p>
              <p className="text-lg">{metrics.overshoot.toFixed(2)} %</p>
            </div>
            <div className="text-center p-2 bg-gray-100 rounded">
              <p className="text-sm font-medium">Settling Time</p>
              <p className="text-lg">{metrics.settlingTime.toFixed(2)} s</p>
            </div>
          </div>
          <div className="mt-4">
            <h3 className="font-medium">Current Score: {calculateScore().toFixed(2)}</h3>
            <p className="text-xs text-gray-600">Lower score is better. Score is weighted sum of rise time, overshoot, and settling time.</p>
          </div>
        </div>
      </div>
      
      <div className="mb-6">
        <h2 className="text-lg font-semibold mb-2">Altitude Response</h2>
        <ResponsiveContainer width="100%" height={300}>
          <LineChart
            data={data}
            margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
          >
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" label={{ value: 'Time (s)', position: 'insideBottomRight', offset: 0 }} />
            <YAxis label={{ value: 'Altitude (m)', angle: -90, position: 'insideLeft' }} />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="altitude" stroke="#8884d8" dot={false} name="Altitude" />
            <Line type="monotone" dataKey="setpoint" stroke="#ff7300" dot={false} strokeDasharray="5 5" name="Setpoint" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      
      <div className="mb-6">
        <h2 className="text-lg font-semibold mb-2">Control Effort (Thrust)</h2>
        <ResponsiveContainer width="100%" height={300}>
          <LineChart
            data={data}
            margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
          >
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" label={{ value: 'Time (s)', position: 'insideBottomRight', offset: 0 }} />
            <YAxis label={{ value: 'Thrust (N)', angle: -90, position: 'insideLeft' }} />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="thrust" stroke="#82ca9d" dot={false} name="Thrust" />
            <Line type="monotone" dataKey="error" stroke="#ff0000" dot={false} name="Error" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}