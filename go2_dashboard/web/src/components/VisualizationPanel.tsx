import { useRef } from 'react';
import { useRos } from '../context/RosContext';
import { MapCanvas, type MapCanvasHandle } from './MapCanvas';
import { PointCloudViewer } from './PointCloudViewer';

export function VisualizationPanel() {
  const { mode, setMode, robot, goal, cancelNavigation } = useRos();
  const mapRef = useRef<MapCanvasHandle>(null);

  const hint =
    mode === 'nav'
      ? 'Click 2D map to send a goal · scroll to zoom · shift+drag to pan'
      : 'Click 2D map to set initial pose (AMCL) · scroll to zoom · shift+drag to pan';

  return (
    <section className="card viz-panel">
      <div className="panel-header">
        <h2>Visualization</h2>
        <div className="toolbar">
          <button
            type="button"
            className="btn btn-ghost"
            onClick={() => mapRef.current?.zoomOut()}
            title="Zoom out"
          >
            −
          </button>
          <button
            type="button"
            className="btn btn-ghost"
            onClick={() => mapRef.current?.zoomIn()}
            title="Zoom in"
          >
            +
          </button>
          <button
            type="button"
            className="btn btn-ghost"
            onClick={() => mapRef.current?.fit()}
            title="Fit map to view"
          >
            Fit
          </button>
          <button
            type="button"
            className={`btn btn-ghost ${mode === 'pose' ? 'active' : ''}`}
            onClick={() => setMode('pose')}
            title="Click map to set initial pose"
          >
            Set Pose
          </button>
          <button
            type="button"
            className={`btn btn-ghost ${mode === 'nav' ? 'active' : ''}`}
            onClick={() => setMode('nav')}
            title="Click map to send navigation goal"
          >
            Go To
          </button>
          <button type="button" className="btn btn-danger" onClick={cancelNavigation}>
            Cancel
          </button>
        </div>
      </div>

      <div className="viz-split">
        <div className="viz-pane">
          <div className="viz-pane-label">2D Occupancy Map</div>
          <MapCanvas ref={mapRef} />
          <div className="map-hint">{hint}</div>
        </div>
        <div className="viz-pane">
          <div className="viz-pane-label">3D LiDAR Point Cloud</div>
          <PointCloudViewer />
        </div>
      </div>

      <div className="map-meta">
        <span>
          Robot: ({robot.x.toFixed(2)}, {robot.y.toFixed(2)})
        </span>
        <span>
          Goal:{' '}
          {goal ? `(${goal.x.toFixed(2)}, ${goal.y.toFixed(2)})` : '—'}
        </span>
      </div>
    </section>
  );
}
