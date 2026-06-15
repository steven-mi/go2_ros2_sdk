import { useState } from 'react';
import { useRos } from '../context/RosContext';

export function ControlsCard() {
  const { saveMap, serializeMap } = useRos();
  const [mapName, setMapName] = useState('apartment');

  return (
    <section className="card controls-card">
      <div className="panel-header">
        <h2>Controls</h2>
      </div>
      <div className="control-grid">
        <label className="field">
          <span>Map name (save)</span>
          <input
            type="text"
            value={mapName}
            onChange={(e) => setMapName(e.target.value)}
            placeholder="apartment"
          />
        </label>
        <button
          type="button"
          className="btn btn-primary full"
          onClick={() => saveMap(mapName.trim() || 'apartment')}
        >
          Save Map
        </button>
        <button
          type="button"
          className="btn btn-secondary full"
          onClick={() => serializeMap(mapName.trim() || 'apartment')}
        >
          Serialize Map
        </button>
      </div>
      <p className="help">
        Save map while running <strong>mapping.launch.py</strong>. Use{' '}
        <strong>navigation.launch.py</strong> with the saved <code>.yaml</code> for Go To mode.
      </p>
    </section>
  );
}
