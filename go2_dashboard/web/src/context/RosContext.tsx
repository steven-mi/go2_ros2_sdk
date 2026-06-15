import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from 'react';
import { Ros, Service, ServiceRequest, Topic } from 'roslib';
import {
  applyMapUpdate,
  normalizeMapMessage,
  quatToYaw,
  yawToQuat,
} from '../lib/mapUtils';
import type {
  GoalPose,
  InteractionMode,
  OccupancyGrid,
  OccupancyGridUpdate,
  OdometryMessage,
  PoseWithCovarianceStamped,
  RobotPose,
} from '../lib/rosMessages';

interface DetectedObjectSummary {
  classId: string;
  score: number;
  localized: boolean;
}

interface RosContextValue {
  connected: boolean;
  rosHost: string;
  rosPort: string;
  setRosHost: (host: string) => void;
  setRosPort: (port: string) => void;
  reconnect: () => void;
  ros: Ros | null;
  mode: InteractionMode;
  setMode: (mode: InteractionMode) => void;
  map: OccupancyGrid | null;
  mapVersion: number;
  robot: RobotPose;
  goal: GoalPose | null;
  hasAmcl: boolean;
  setGoal: (goal: GoalPose | null) => void;
  publishGoal: (x: number, y: number) => void;
  publishInitialPose: (x: number, y: number) => void;
  cancelNavigation: () => void;
  saveMap: (name: string) => void;
  serializeMap: (name: string) => void;
  navigateToObject: (
    classId: string,
    standoffDistance: number,
    onResult: (message: string, success: boolean) => void,
  ) => void;
  listDetectedObjects: (onResult: (items: DetectedObjectSummary[]) => void) => void;
}

const RosContext = createContext<RosContextValue | null>(null);

const defaultHost =
  typeof window !== 'undefined' ? window.location.hostname || 'localhost' : 'localhost';

function buildRosBridgeUrl(host: string, port: string): string {
  if (import.meta.env.DEV) {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${window.location.host}/rosbridge`;
  }
  const trimmedHost = host.trim() || 'localhost';
  const trimmedPort = port.trim() || '9090';
  return `ws://${trimmedHost}:${trimmedPort}`;
}

export function RosProvider({ children }: { children: ReactNode }) {
  const [connected, setConnected] = useState(false);
  const [rosHost, setRosHost] = useState(defaultHost);
  const [rosPort, setRosPort] = useState('9090');
  const [ros, setRos] = useState<Ros | null>(null);
  const [mode, setMode] = useState<InteractionMode>('nav');
  const [map, setMap] = useState<OccupancyGrid | null>(null);
  const [mapVersion, setMapVersion] = useState(0);
  const [robot, setRobot] = useState<RobotPose>({ x: 0, y: 0, yaw: 0 });
  const [goal, setGoal] = useState<GoalPose | null>(null);
  const [hasAmcl, setHasAmcl] = useState(false);

  const topicsRef = useRef<Record<string, Topic>>({});
  const servicesRef = useRef<Record<string, Service>>({});
  const rosRef = useRef<Ros | null>(null);
  const rosHostRef = useRef(rosHost);
  const rosPortRef = useRef(rosPort);
  const hasAmclRef = useRef(false);
  const robotRef = useRef(robot);

  useEffect(() => {
    rosHostRef.current = rosHost;
  }, [rosHost]);

  useEffect(() => {
    rosPortRef.current = rosPort;
  }, [rosPort]);

  useEffect(() => {
    robotRef.current = robot;
  }, [robot]);

  const sendRosbridgeMessage = useCallback((payload: Record<string, unknown>) => {
    if (!rosRef.current?.isConnected) return;
    rosRef.current.socket.send(JSON.stringify(payload));
  }, []);

  const sendNavigateGoal = useCallback(
    (poseMsg: {
      header: { frame_id: string; stamp: { sec: number; nanosec: number } };
      pose: {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
      };
    }) => {
      sendRosbridgeMessage({
        op: 'send_action_goal',
        id: `nav_goal_${Date.now()}`,
        action: '/navigate_to_pose',
        action_type: 'nav2_msgs/action/NavigateToPose',
        args: { pose: poseMsg },
        feedback: false,
      });
    },
    [sendRosbridgeMessage],
  );

  const publishGoal = useCallback(
    (x: number, y: number) => {
      const { x: rx, y: ry } = robotRef.current;
      const yaw = Math.atan2(y - ry, x - rx);
      const msg = {
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x, y, z: 0 },
          orientation: yawToQuat(yaw),
        },
      };

      setGoal({ x, y });
      topicsRef.current.goal?.publish(msg);
      sendNavigateGoal(msg);
    },
    [sendNavigateGoal],
  );

  const publishInitialPose = useCallback((x: number, y: number) => {
    const msg = {
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        pose: {
          position: { x, y, z: 0 },
          orientation: yawToQuat(robotRef.current.yaw),
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0.06853891945200942,
        ],
      },
    };
    topicsRef.current.initialPose?.publish(msg);
    setRobot((prev) => ({ ...prev, x, y }));
  }, []);

  const cancelNavigation = useCallback(() => {
    sendRosbridgeMessage({
      op: 'cancel_action_goal',
      id: `nav_cancel_${Date.now()}`,
      action: '/navigate_to_pose',
    });
    setGoal(null);
  }, [sendRosbridgeMessage]);

  const callSaveMap = useCallback((serviceKey: string, name: string) => {
    const service = servicesRef.current[serviceKey];
    if (!service) return;
    const req = new ServiceRequest({ name: `/ros2_ws/data/${name}` });
    service.callService(req, () => {}, () => {});
  }, []);

  const navigateToObject = useCallback(
    (
      classId: string,
      standoffDistance: number,
      onResult: (message: string, success: boolean) => void,
    ) => {
      const service = servicesRef.current.navigateToObject;
      if (!service) {
        onResult('navigate_to_object service unavailable', false);
        return;
      }
      const req = new ServiceRequest({
        class_id: classId,
        standoff_distance: standoffDistance,
      });
      service.callService(
        req,
        (res: { success?: boolean; message?: string; goal_pose?: { pose?: { position?: { x: number; y: number } } } }) => {
          const success = Boolean(res.success);
          const message = res.message ?? (success ? 'Navigation started' : 'Navigation failed');
          if (success && res.goal_pose?.pose?.position) {
            setGoal({
              x: res.goal_pose.pose.position.x,
              y: res.goal_pose.pose.position.y,
            });
          }
          onResult(message, success);
        },
        () => onResult('navigate_to_object service call failed', false),
      );
    },
    [],
  );

  const listDetectedObjects = useCallback((onResult: (items: DetectedObjectSummary[]) => void) => {
    const service = servicesRef.current.listDetectedObjects;
    if (!service) {
      onResult([]);
      return;
    }
    service.callService(
      new ServiceRequest({}),
      (res: {
        class_ids?: string[];
        scores?: number[];
        localized?: boolean[];
      }) => {
        const classIds = res.class_ids ?? [];
        const scores = res.scores ?? [];
        const localized = res.localized ?? [];
        onResult(
          classIds.map((classId, index) => ({
            classId,
            score: scores[index] ?? 0,
            localized: localized[index] ?? false,
          })),
        );
      },
      () => onResult([]),
    );
  }, []);

  const teardown = useCallback(() => {
    Object.values(topicsRef.current).forEach((topic) => topic.unsubscribe());
    topicsRef.current = {};
    servicesRef.current = {};
    const activeRos = rosRef.current;
    rosRef.current = null;
    if (!activeRos) {
      return;
    }
    try {
      const socket = (activeRos as Ros & { socket?: WebSocket }).socket;
      if (socket?.readyState === WebSocket.CONNECTING) {
        socket.onopen = () => socket.close();
        return;
      }
      activeRos.close();
    } catch {
      /* ignore */
    }
  }, []);

  const setupRos = useCallback(() => {
    teardown();

    hasAmclRef.current = false;
    setHasAmcl(false);
    setMap(null);
    setMapVersion(0);
    setGoal(null);
    setConnected(false);

    const host = rosHostRef.current.trim() || 'localhost';
    const port = rosPortRef.current.trim() || '9090';
    const url = buildRosBridgeUrl(host, port);
    const nextRos = new Ros({ url });
    rosRef.current = nextRos;

    nextRos.on('connection', () => setConnected(true));
    nextRos.on('close', () => setConnected(false));
    nextRos.on('error', () => setConnected(false));

    const subscribeMapTopics = () => {
      topicsRef.current.map?.unsubscribe();
      topicsRef.current.mapUpdates?.unsubscribe();

      topicsRef.current.map = new Topic({
        ros: nextRos,
        name: '/map',
        messageType: 'nav_msgs/msg/OccupancyGrid',
      });
      topicsRef.current.map.subscribe((msg) => {
        setMap(normalizeMapMessage(msg as { info: OccupancyGrid['info']; data: number[] | string }));
        setMapVersion((v) => v + 1);
      });

      topicsRef.current.mapUpdates = new Topic({
        ros: nextRos,
        name: '/map_updates',
        messageType: 'map_msgs/msg/OccupancyGridUpdate',
      });
      topicsRef.current.mapUpdates.subscribe((update) => {
        setMap((prev) => {
          if (!prev) return prev;
          return applyMapUpdate(prev, update as OccupancyGridUpdate);
        });
        setMapVersion((v) => v + 1);
      });
    };

    // Rosbridge picks TRANSIENT_LOCAL for /map only if slam_toolbox is already publishing.
    [0, 3000, 8000].forEach((delay) => window.setTimeout(subscribeMapTopics, delay));

    topicsRef.current.amcl = new Topic({
      ros: nextRos,
      name: '/amcl_pose',
      messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    });
    topicsRef.current.amcl.subscribe((msg) => {
      hasAmclRef.current = true;
      setHasAmcl(true);
      const pose = (msg as PoseWithCovarianceStamped).pose.pose;
      setRobot({
        x: pose.position.x,
        y: pose.position.y,
        yaw: quatToYaw(pose.orientation),
      });
    });

    topicsRef.current.odom = new Topic({
      ros: nextRos,
      name: '/odom',
      messageType: 'nav_msgs/msg/Odometry',
    });
    topicsRef.current.odom.subscribe((msg) => {
      if (hasAmclRef.current) return;
      const pose = (msg as OdometryMessage).pose.pose;
      setRobot({
        x: pose.position.x,
        y: pose.position.y,
        yaw: quatToYaw(pose.orientation),
      });
    });

    topicsRef.current.goal = new Topic({
      ros: nextRos,
      name: '/goal_pose',
      messageType: 'geometry_msgs/msg/PoseStamped',
    });

    topicsRef.current.initialPose = new Topic({
      ros: nextRos,
      name: '/initialpose',
      messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    });

    servicesRef.current.saveMap = new Service({
      ros: nextRos,
      name: '/slam_toolbox/save_map',
      serviceType: 'slam_toolbox/srv/SaveMap',
    });

    servicesRef.current.serializeMap = new Service({
      ros: nextRos,
      name: '/slam_toolbox/serialize_map',
      serviceType: 'slam_toolbox/srv/SaveMap',
    });

    servicesRef.current.navigateToObject = new Service({
      ros: nextRos,
      name: '/navigate_to_object',
      serviceType: 'go2_interfaces/srv/NavigateToObject',
    });

    servicesRef.current.listDetectedObjects = new Service({
      ros: nextRos,
      name: '/list_detected_objects',
      serviceType: 'go2_interfaces/srv/ListDetectedObjects',
    });

    setRos(nextRos);
  }, [teardown]);

  useEffect(() => {
    setupRos();
    return teardown;
    // Connect once on mount; use Reconnect for host/port changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const value = useMemo<RosContextValue>(
    () => ({
      connected,
      rosHost,
      rosPort,
      setRosHost,
      setRosPort,
      reconnect: setupRos,
      ros,
      mode,
      setMode,
      map,
      mapVersion,
      robot,
      goal,
      hasAmcl,
      setGoal,
      publishGoal,
      publishInitialPose,
      cancelNavigation,
      saveMap: (name) => callSaveMap('saveMap', name),
      serializeMap: (name) => callSaveMap('serializeMap', name),
      navigateToObject,
      listDetectedObjects,
    }),
    [
      connected,
      rosHost,
      rosPort,
      setupRos,
      ros,
      mode,
      map,
      mapVersion,
      robot,
      goal,
      hasAmcl,
      publishGoal,
      publishInitialPose,
      cancelNavigation,
      callSaveMap,
      navigateToObject,
      listDetectedObjects,
    ],
  );

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>;
}

export function useRos() {
  const ctx = useContext(RosContext);
  if (!ctx) throw new Error('useRos must be used within RosProvider');
  return ctx;
}
