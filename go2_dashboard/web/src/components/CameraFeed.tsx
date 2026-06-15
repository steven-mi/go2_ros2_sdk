import { useEffect, useState } from 'react';
import { Topic } from 'roslib';
import { useRos } from '../context/RosContext';
import { compressedImageSrc, rawImageSrc } from '../lib/cameraUtils';
import type { CompressedImageMessage, ImageMessage } from '../lib/rosMessages';

export function CameraFeed() {
  const { ros, connected } = useRos();
  const [src, setSrc] = useState('');

  useEffect(() => {
    if (!ros || !connected) {
      setSrc('');
      return;
    }

    let hasCompressed = false;

    const compressed = new Topic({
      ros,
      name: '/camera/compressed',
      messageType: 'sensor_msgs/msg/CompressedImage',
    });

    const raw = new Topic({
      ros,
      name: '/camera/image_raw',
      messageType: 'sensor_msgs/msg/Image',
      throttle_rate: 200,
    });

    compressed.subscribe((msg) => {
      hasCompressed = true;
      const next = compressedImageSrc(msg as CompressedImageMessage);
      if (next) setSrc(next);
    });

    raw.subscribe((msg) => {
      if (hasCompressed) return;
      const next = rawImageSrc(msg as ImageMessage);
      if (next) setSrc(next);
    });

    return () => {
      compressed.unsubscribe();
      raw.unsubscribe();
    };
  }, [ros, connected]);

  return (
    <section className="card camera-card">
      <div className="panel-header">
        <h2>Camera</h2>
      </div>
      <div className="camera-wrap">
        {src ? (
          <img className="camera-feed" src={src} alt="Camera feed" />
        ) : (
          <div className="camera-placeholder">Waiting for /camera/compressed…</div>
        )}
      </div>
    </section>
  );
}
