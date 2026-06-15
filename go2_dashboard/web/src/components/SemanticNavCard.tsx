import { useCallback, useState } from 'react';
import { useRos } from '../context/RosContext';

const DEFAULT_CLASSES = ['person', 'chair', 'couch', 'bed', 'dining table'];

export function SemanticNavCard() {
  const { connected, navigateToObject, listDetectedObjects } = useRos();
  const [classId, setClassId] = useState('person');
  const [standoff, setStandoff] = useState('1.0');
  const [status, setStatus] = useState('');
  const [visible, setVisible] = useState<string[]>([]);

  const handleNavigate = useCallback(() => {
    const distance = Number.parseFloat(standoff);
    navigateToObject(
      classId.trim(),
      Number.isFinite(distance) && distance > 0 ? distance : 1.0,
      (message, success) => setStatus(success ? message : `Error: ${message}`),
    );
  }, [classId, navigateToObject, standoff]);

  const handleRefresh = useCallback(() => {
    listDetectedObjects((items) => {
      if (items.length === 0) {
        setVisible([]);
        setStatus('No localized objects in view.');
        return;
      }
      setVisible(items.map((item) => `${item.classId} (${(item.score * 100).toFixed(0)}%)`));
      setStatus(`Visible: ${items.map((item) => item.classId).join(', ')}`);
    });
  }, [listDetectedObjects]);

  return (
    <section className="card controls-card">
      <div className="panel-header">
        <h2>Object Navigation</h2>
      </div>
      <div className="control-grid">
        <label className="field">
          <span>Object class</span>
          <input
            list="coco-classes"
            type="text"
            value={classId}
            onChange={(e) => setClassId(e.target.value)}
            placeholder="person"
          />
          <datalist id="coco-classes">
            {DEFAULT_CLASSES.map((item) => (
              <option key={item} value={item} />
            ))}
          </datalist>
        </label>
        <label className="field">
          <span>Standoff (m)</span>
          <input
            type="number"
            min="0.3"
            step="0.1"
            value={standoff}
            onChange={(e) => setStandoff(e.target.value)}
          />
        </label>
        <button
          type="button"
          className="btn btn-primary full"
          disabled={!connected}
          onClick={handleNavigate}
        >
          Navigate To Object
        </button>
        <button
          type="button"
          className="btn btn-secondary full"
          disabled={!connected}
          onClick={handleRefresh}
        >
          List Visible Objects
        </button>
      </div>
      {visible.length > 0 && (
        <ul className="help">
          {visible.map((item) => (
            <li key={item}>{item}</li>
          ))}
        </ul>
      )}
      {status && <p className="help">{status}</p>}
      <p className="help">
        Uses COCO detections + LiDAR to localize objects, then sends a Nav2 goal with standoff.
      </p>
    </section>
  );
}
