import { useRos } from '../context/RosContext';

export function ConnectionCard() {
  const { rosHost, rosPort, setRosHost, setRosPort, reconnect } = useRos();

  return (
    <section className="card settings-card">
      <div className="panel-header">
        <h2>Connection</h2>
      </div>
      <div className="settings-body">
        <label className="field">
          <span>ROS bridge host</span>
          <input
            type="text"
            value={rosHost}
            onChange={(e) => setRosHost(e.target.value)}
          />
        </label>
        <label className="field">
          <span>ROS bridge port</span>
          <input
            type="number"
            value={rosPort}
            onChange={(e) => setRosPort(e.target.value)}
          />
        </label>
        <button type="button" className="btn btn-secondary full" onClick={reconnect}>
          Reconnect
        </button>
      </div>
    </section>
  );
}
