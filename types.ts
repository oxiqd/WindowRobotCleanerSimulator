
export type CleaningMode = 1 | 2 | 3;
export type RotationDirection = 'CLOCKWISE' | 'COUNTERCLOCKWISE';
export type MovementDirection = 'UP' | 'DOWN' | 'LEFT' | 'RIGHT';
export type RobotStatus = 'IDLE' | 'CLEANING' | 'PAUSED' | 'COMPLETED' | 'ERROR';
export type DiskSpeed = number; // Degrees per second.
export type DiskAngle = number; // Degrees.
export type Disk = 'TOP' | 'BOTTOM';
export type Pose = { x: number; y: number; angleDeg: number };

export type MotorState = {
  running: boolean;
  direction: RotationDirection | null;
  speedDegPerSec: number;
  totalRotationDeg: number;
};

export type RobotState = {
  pose: Pose;
  status: RobotStatus;
  direction: MovementDirection;
  mode: CleaningMode | null;
  vacuumOn: boolean;
  pressure: number; // Simulated sensor value from 0 to 100.
  isMounted: boolean;
  isMoving: boolean;
  isObstacle: boolean;
  obstacleDirection: MovementDirection | null;
  motors: { top: MotorState; bottom: MotorState };
  virtualSeconds: number;
  completedSteps: number;
  cleanedCells: Set<string>;
};

export type WindowConfig = {
  widthCm: number;
  heightCm: number;
  discRadiusCm: number;
  discSpacingCm: number;
  minimumPressure: number;
  gridCm: number;
  horizontalStepAngleDeg: number;
  verticalStepAngleDeg: number;
  motorSpeedDegPerSec: number;
};

export type SensorReadings = {
  pressure: number;
  vacuumSafe: boolean;
  obstacleAhead: boolean;
  movementDirection: MovementDirection;
};

export type MotorCommand = {
  disk: Disk;
  direction: RotationDirection;
  angle: number;
  speed: number;
};

export type RobotEvent = {
  time: number;
  type: string;
  message: string;
};