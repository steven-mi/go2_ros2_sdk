import { CameraFeed } from './components/CameraFeed';
import { ConnectionCard } from './components/ConnectionCard';
import { ControlsCard } from './components/ControlsCard';
import { SemanticNavCard } from './components/SemanticNavCard';
import { TopBar } from './components/TopBar';
import { VisualizationPanel } from './components/VisualizationPanel';
import { RosProvider } from './context/RosContext';
import './App.css';

export default function App() {
  return (
    <RosProvider>
      <div className="app">
        <TopBar />
        <main className="layout">
          <VisualizationPanel />
          <aside className="sidebar">
            <CameraFeed />
            <SemanticNavCard />
            <ControlsCard />
            <ConnectionCard />
          </aside>
        </main>
      </div>
    </RosProvider>
  );
}
