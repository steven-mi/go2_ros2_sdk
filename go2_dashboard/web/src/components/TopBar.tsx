import { useRos } from '../context/RosContext';

export function TopBar() {
  const { connected } = useRos();

  return (
    <header className="topbar">
      <div className="brand">
        <span className="brand-icon">🐕</span>
        <div>
          <h1>Go2 Dashboard</h1>
          <p>{connected ? 'Live map & LiDAR · click to navigate' : 'Waiting for rosbridge…'}</p>
        </div>
      </div>
      <div className={`status-pill ${connected ? 'connected' : ''}`}>
        <span className="dot" />
        <span>{connected ? 'Connected' : 'Disconnected'}</span>
      </div>
    </header>
  );
}
