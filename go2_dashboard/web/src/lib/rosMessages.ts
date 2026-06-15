export type InteractionMode = 'nav' | 'pose';

export interface MapInfo {
  width: number;
  height: number;
  resolution: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

export interface OccupancyGrid {
  info: MapInfo;
  data: number[];
}

export interface OccupancyGridUpdate {
  x: number;
  y: number;
  width: number;
  height: number;
  data: number[] | string;
}

export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

export interface GoalPose {
  x: number;
  y: number;
}

export interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export interface PointCloud2Message {
  header: { frame_id: string };
  height: number;
  width: number;
  fields: PointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: number[] | string;
  is_dense: boolean;
}

export interface CompressedImageMessage {
  format?: string;
  data: number[] | string;
}

export interface ImageMessage {
  encoding?: string;
  width: number;
  height: number;
  data: number[] | string;
}

export interface PoseWithCovarianceStamped {
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}

export interface OdometryMessage {
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}
