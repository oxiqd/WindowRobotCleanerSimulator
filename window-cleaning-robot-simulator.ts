
/**
 * Window-cleaning robot simulator.
 * The walking model is an educational approximation, not a mechanical model
 * of a particular commercial robot. All dimensions are in centimetres.
 * Run: tsc window-robot-simulator.ts --target es2022 --module commonjs && node window-robot-simulator.js
 */

import type {
  CleaningMode,
  RotationDirection,
  MovementDirection,
  DiskSpeed,
  DiskAngle,
  Disk,
  Pose,
  RobotState,
  WindowConfig,
  SensorReadings,
  MotorCommand,
  RobotEvent,
} from "./types.ts";


class WindowRobotSimulator {
  // Each instance owns its own mutable state. No static state is shared.
  readonly config: WindowConfig;
  readonly state: RobotState;
  readonly events: RobotEvent[] = [];
  private readonly quietMotorLogs: boolean;

  constructor(config: WindowConfig, start: Pose, quietMotorLogs = false) {
    this.config = config;
    this.quietMotorLogs = quietMotorLogs;
    this.state = {
      pose: { ...start },
      status: 'IDLE',
      direction: 'RIGHT',
      mode: null,
      vacuumOn: false,
      pressure: 0,
      isMounted: false,
      isMoving: false,
      isObstacle: false,
      obstacleDirection: null,
      motors: {
        top: { running: false, direction: null, speedDegPerSec: 0, totalRotationDeg: 0 },
        bottom: { running: false, direction: null, speedDegPerSec: 0, totalRotationDeg: 0 },
      },
      virtualSeconds: 0,
      completedSteps: 0,
      cleanedCells: new Set<string>(),
    };
    if (!this.isPoseWithinWindow(start)) {
      throw new Error('The initial robot pose intersects the window frame.');
    }
  }

  // --- 1. Robot controls ---

  powerOn(): void {
    // Turn on the suction turbine and check whether cleaning can start.
    if (this.state.vacuumOn) return;
    this.updateRobotState({ vacuumOn: true, pressure: 92, isMounted: true });
    this.logEvent('POWER', 'Vacuum ON; pressure=92/100');
    if (!this.checkVacuum()) this.emergencyStop('Insufficient pressure on startup');
  }

  powerOff(supportedByUser = false): void {
    // Turn off suction only when the user is physically supporting the robot.
    if (this.state.isMounted && !supportedByUser) {
      this.logEvent('WARNING', 'Refused to turn vacuum off while robot is mounted.');
      return;
    }
    this.stopMotors();
    this.updateRobotState({ vacuumOn: false, pressure: 0, isMounted: false, status: 'IDLE' });
    this.logEvent('POWER', 'Vacuum OFF');
  }

  startCleaning(mode: CleaningMode): void {
    // Choose a cleaning pattern and start an autonomous run.
    if (!this.checkVacuum()) {
      this.emergencyStop('Cannot start cleaning without safe vacuum');
      return;
    }
    if (this.state.status === 'CLEANING') {
      this.logEvent('WARNING', 'Cleaning is already active.');
      return;
    }
    this.updateRobotState({
      mode,
      direction: mode === 2 ? 'LEFT' : 'RIGHT',
      status: 'CLEANING',
      isObstacle: false,
      obstacleDirection: null,
    });
    this.updateCleanedArea();
    this.logEvent('START', `Mode ${mode}; initial direction ${this.state.direction}`);
  }

  stopCleaning(): void {
    // Stop cleaning motion while keeping the suction turbine running.
    this.stopMotors();
    this.updateRobotState({ status: 'IDLE', isMoving: false });
    this.logEvent('STOP', 'Cleaning stopped; vacuum remains ON');
  }

  pauseCleaning(): void {
    // Suspend cleaning without losing the current position or mode.
    if (this.state.status !== 'CLEANING') return;
    this.stopMotors();
    this.updateRobotState({ status: 'PAUSED', isMoving: false });
    this.logEvent('PAUSE', 'Cleaning paused');
  }

  resumeCleaning(): void {
    // Resume cleaning from the stored state when pressure is safe.
    if (this.state.status !== 'PAUSED') return;
    if (!this.checkVacuum()) return this.emergencyStop('Pressure too low to resume');
    this.updateRobotState({ status: 'CLEANING' });
    this.logEvent('RESUME', 'Cleaning resumed');
  }

  // --- 2. Disk motor controls ---

  rotateTopDisk(direction: RotationDirection, angle: DiskAngle, speed: DiskSpeed): boolean {
    // Rotate the top drive; approximate the chassis pivoting around the bottom disk.
    return this.rotateDisk('TOP', direction, angle, speed);
  }

  rotateBottomDisk(direction: RotationDirection, angle: DiskAngle, speed: DiskSpeed): boolean {
    // Rotate the bottom drive; approximate the chassis pivoting around the top disk.
    return this.rotateDisk('BOTTOM', direction, angle, speed);
  }

  stopMotors(): void {
    // Stop both movement motors without changing the vacuum turbine state.
    for (const motor of [this.state.motors.top, this.state.motors.bottom]) {
      motor.running = false;
      motor.direction = null;
      motor.speedDegPerSec = 0;
    }
    this.state.isMoving = false;
  }

  // --- 3. Movement mechanics ---

  moveRight(): boolean {
    // Perform one mechanically simulated movement cycle to the right.
    return this.performMovement('RIGHT');
  }

  moveLeft(): boolean {
    // Perform one mechanically simulated movement cycle to the left.
    return this.performMovement('LEFT');
  }

  moveUp(): boolean {
    // Perform one mechanically simulated movement cycle upward.
    return this.performMovement('UP');
  }

  moveDown(): boolean {
    // Perform one mechanically simulated movement cycle downward.
    return this.performMovement('DOWN');
  }

  moveForward(): boolean {
    // Move one cycle in the current direction selected by the controller.
    switch (this.state.direction) {
      case 'RIGHT': return this.moveRight();
      case 'LEFT': return this.moveLeft();
      case 'UP': return this.moveUp();
      case 'DOWN': return this.moveDown();
    }
  }

  updatePosition(disk: Disk, signedAngleDeg: number): boolean {
    // Calculate the next chassis pose around the opposite disk's fixed contact point.
    const samples = this.sampleRotation(this.state.pose, disk, signedAngleDeg);
    if (samples.some((sample) => !this.isPoseWithinWindow(sample))) return false;
    this.state.pose = samples[samples.length - 1] ?? this.state.pose;
    this.updateCleanedArea(samples);
    return true;
  }

  // --- 4. Sensors and safety ---

  readSensors(): SensorReadings {
    // Read simulated suction and edge sensors; the controller does not receive a map.
    return {
      pressure: this.state.pressure,
      vacuumSafe: this.checkVacuum(),
      obstacleAhead: this.detectObstacle(this.state.direction),
      movementDirection: this.state.direction,
    };
  }

  checkVacuum(): boolean {
    // Check simulated suction against the required safety threshold.
    return this.state.vacuumOn && this.state.pressure >= this.config.minimumPressure;
  }

  detectObstacle(direction: MovementDirection): boolean {
    // Check if the next full disk-motion cycle would intersect the window frame.
    let pose = { ...this.state.pose };
    for (const command of this.movementPlan(direction)) {
      const delta = command.direction === 'CLOCKWISE' ? command.angle : -command.angle;
      for (const sample of this.sampleRotation(pose, command.disk, delta)) {
        if (!this.isPoseWithinWindow(sample)) return true;
      }
      const samples = this.sampleRotation(pose, command.disk, delta);
      pose = samples[samples.length - 1] ?? pose;
    }
    return false;
  }

  checkMovement(before: Pose): boolean {
    // Verify that a completed motor cycle produced actual translation.
    const delta = Math.hypot(this.state.pose.x - before.x, this.state.pose.y - before.y);
    if (delta < 0.1) {
      this.logEvent('ERROR', 'Motor cycle completed without meaningful movement');
      return false;
    }
    return true;
  }

  emergencyStop(reason: string): void {
    // Stop movement, command the suction turbine to stay on, and report unsafe state.
    this.stopMotors();
    this.updateRobotState({ status: 'ERROR', isMoving: false });
    this.logEvent('EMERGENCY', reason);
  }

  // --- 5. Autonomous cleaning algorithm ---

  cleanWindow(mode: CleaningMode, maxCycles = 3000): void {
    // Run the autonomous controller until completion, failure, or the safety limit.
    this.startCleaning(mode);
    let cycles = 0;
    while (this.state.status === 'CLEANING' && cycles < maxCycles) {
      this.simulationStep();
      cycles += 1;
    }
    if (cycles >= maxCycles && this.state.status === 'CLEANING') {
      this.emergencyStop('Controller cycle limit exceeded');
    }
  }

  cleanRow(): void {
    // Advance along the current row; transition when a side frame is detected.
    if (this.state.status !== 'CLEANING') return;
    const sensors = this.readSensors();
    if (!sensors.vacuumSafe) return this.emergencyStop('Vacuum lost during cleaning');
    if (sensors.obstacleAhead) {
      this.state.isObstacle = true;
      this.state.obstacleDirection = this.state.direction;
      this.logEvent('EDGE', `${this.state.direction} boundary detected`);
      if (!this.changeRow()) return this.finishCleaning();
      this.changeDirection();
      return;
    }
    this.moveForward();
  }

  changeDirection(): void {
    // Reverse the horizontal movement direction after changing rows.
    const next = this.state.direction === 'RIGHT' ? 'LEFT' : 'RIGHT';
    this.updateRobotState({ direction: next, isObstacle: false, obstacleDirection: null });
    this.logEvent('DIRECTION', `Now moving ${next}`);
  }

  changeRow(): boolean {
    // Shift one cleaning strip downward, or upward in the third mode.
    const vertical: MovementDirection = this.state.mode === 3 ? 'UP' : 'DOWN';
    if (this.detectObstacle(vertical)) {
      this.logEvent('EDGE', `${vertical} boundary detected; no further rows`);
      return false;
    }
    if (!this.performMovement(vertical)) return false;
    this.logEvent('ROW', `Shifted ${vertical} to y=${this.state.pose.y.toFixed(2)} cm`);
    return true;
  }

  finishCleaning(): void {
    // End the autonomous route and preserve suction for safe manual removal.
    this.stopMotors();
    this.updateRobotState({ status: 'COMPLETED', isMoving: false });
    this.logEvent('FINISH', `Route complete; ${this.coveragePercent().toFixed(1)}% grid coverage`);
  }

  // --- 6. State, simulation, visualization and event log ---

  updateRobotState(patch: Partial<RobotState>): void {
    // Apply a typed state update without replacing the whole robot instance.
    Object.assign(this.state, patch);
  }

  updateCleanedArea(poses: Pose[] = [this.state.pose]): void {
    // Mark all grid cells swept by the two circular cleaning pads.
    const step = this.config.gridCm;
    const columns = Math.ceil(this.config.widthCm / step);
    const rows = Math.ceil(this.config.heightCm / step);
    for (const pose of poses) {
      for (const disk of ['TOP', 'BOTTOM'] as const) {
        const center = this.diskCenter(pose, disk);
        const minX = Math.max(0, Math.floor((center.x - this.config.discRadiusCm) / step));
        const maxX = Math.min(columns - 1, Math.floor((center.x + this.config.discRadiusCm) / step));
        const minY = Math.max(0, Math.floor((center.y - this.config.discRadiusCm) / step));
        const maxY = Math.min(rows - 1, Math.floor((center.y + this.config.discRadiusCm) / step));
        for (let y = minY; y <= maxY; y += 1) {
          for (let x = minX; x <= maxX; x += 1) {
            const cellX = Math.min((x + 0.5) * step, this.config.widthCm);
            const cellY = Math.min((y + 0.5) * step, this.config.heightCm);
            if (Math.hypot(cellX - center.x, cellY - center.y) <= this.config.discRadiusCm) {
              this.state.cleanedCells.add(`${x}:${y}`);
            }
          }
        }
      }
    }
  }

  simulationStep(): void {
    // Execute one logical controller cycle; motor rotations consume virtual time.
    if (this.state.status !== 'CLEANING') return;
    if (!this.checkVacuum()) return this.emergencyStop('Unsafe vacuum pressure');
    this.cleanRow();
  }

  renderRobot(): void {
    // Print the current body pose, motor totals and active safety state.
    const { pose, motors, pressure, status, direction } = this.state;
    console.log(`Robot: x=${pose.x.toFixed(2)}cm y=${pose.y.toFixed(2)}cm ` +
      `angle=${pose.angleDeg.toFixed(1)}° direction=${direction} status=${status} pressure=${pressure}`);
    console.log(`Motors: top=${motors.top.totalRotationDeg.toFixed(0)}° ` +
      `bottom=${motors.bottom.totalRotationDeg.toFixed(0)}°`);
  }

  renderWindow(): void {
    // Print an ASCII map of cleaned cells and the current chassis centre.
    const step = this.config.gridCm;
    const width = Math.ceil(this.config.widthCm / step);
    const height = Math.ceil(this.config.heightCm / step);
    const robotX = Math.min(width - 1, Math.floor(this.state.pose.x / step));
    const robotY = Math.min(height - 1, Math.floor(this.state.pose.y / step));
    console.log(`+${'-'.repeat(width)}+`);
    for (let y = 0; y < height; y += 1) {
      let row = '';
      for (let x = 0; x < width; x += 1) {
        row += x === robotX && y === robotY ? 'R'
          : this.state.cleanedCells.has(`${x}:${y}`) ? '.' : ' ';
      }
      console.log(`|${row}|`);
    }
    console.log(`+${'-'.repeat(width)}+`);
    console.log(`Covered grid cells: ${this.coveragePercent().toFixed(1)}%`);
  }

  logEvent(type: string, message: string): void {
    // Store and print an event with simulated elapsed time.
    const event = { time: this.state.virtualSeconds, type, message };
    this.events.push(event);
    if (type !== 'MOTOR' || !this.quietMotorLogs) {
      console.log(`[${event.time.toFixed(1)}s] [${type}] ${message}`);
    }
  }

  // --- Internal helpers: motor sequencing and simplified geometry ---

  private rotateDisk(disk: Disk, direction: RotationDirection, angle: DiskAngle, speed: DiskSpeed): boolean {
    if (!this.checkVacuum() || !this.state.isMounted) {
      this.logEvent('ERROR', 'Motor command rejected: vacuum is not safe');
      return false;
    }
    if (angle <= 0 || speed <= 0) throw new Error('Angle and speed must be positive.');
    const motor = disk === 'TOP' ? this.state.motors.top : this.state.motors.bottom;
    const signedAngle = direction === 'CLOCKWISE' ? angle : -angle;
    motor.running = true;
    motor.direction = direction;
    motor.speedDegPerSec = speed;
    const moved = this.updatePosition(disk, signedAngle);
    motor.running = false;
    motor.direction = null;
    motor.speedDegPerSec = 0;
    if (!moved) return false;
    motor.totalRotationDeg += angle;
    this.state.virtualSeconds += angle / speed;
    this.logEvent('MOTOR', `${disk} ${direction} ${angle}° @ ${speed}°/s; ` +
      `body=(${this.state.pose.x.toFixed(1)},${this.state.pose.y.toFixed(1)}) ` +
      `θ=${this.state.pose.angleDeg.toFixed(1)}°`);
    return true;
  }

  private movementPlan(direction: MovementDirection): MotorCommand[] {
    // Each cycle is four pivot motions. Horizontal cycles cancel vertical drift.
    // Vertical cycles cancel horizontal drift. This is a simplified gait, not real firmware.
    const a = (direction === 'RIGHT' || direction === 'LEFT')
      ? this.config.horizontalStepAngleDeg : this.config.verticalStepAngleDeg;
    const s = this.config.motorSpeedDegPerSec;
    const cmd = (disk: Disk, clockwise: boolean): MotorCommand => ({
      disk, direction: clockwise ? 'CLOCKWISE' : 'COUNTERCLOCKWISE', angle: a, speed: s,
    });
    switch (direction) {
      case 'RIGHT': return [cmd('TOP', true), cmd('BOTTOM', false), cmd('BOTTOM', false), cmd('TOP', true)];
      case 'LEFT': return [cmd('TOP', false), cmd('BOTTOM', true), cmd('BOTTOM', true), cmd('TOP', false)];
      case 'DOWN': {
        const firstClockwise = this.state.direction === 'LEFT';
        return [cmd('TOP', firstClockwise), cmd('BOTTOM', !firstClockwise),
          cmd('TOP', !firstClockwise), cmd('BOTTOM', firstClockwise)];
      }
      case 'UP': {
        const firstClockwise = this.state.direction === 'RIGHT';
        return [cmd('BOTTOM', firstClockwise), cmd('TOP', !firstClockwise),
          cmd('BOTTOM', !firstClockwise), cmd('TOP', firstClockwise)];
      }
    }
  }

  private performMovement(direction: MovementDirection): boolean {
    if (this.state.status !== 'CLEANING' || !this.checkVacuum()) return false;
    if (this.detectObstacle(direction)) {
      this.state.isObstacle = true;
      this.state.obstacleDirection = direction;
      this.logEvent('EDGE', `Blocked movement: ${direction}`);
      return false;
    }
    const before = { ...this.state.pose };
    this.state.isMoving = true;
    for (const command of this.movementPlan(direction)) {
      const moved = command.disk === 'TOP'
        ? this.rotateTopDisk(command.direction, command.angle, command.speed)
        : this.rotateBottomDisk(command.direction, command.angle, command.speed);
      if (!moved) return this.emergencyStopWithFalse(`Unexpected motion blockage while going ${direction}`);
    }
    this.stopMotors();
    if (!this.checkMovement(before)) return this.emergencyStopWithFalse('Movement sensor reported no progress');
    this.state.completedSteps += 1;
    this.logEvent('MOVE', `${direction}: (${before.x.toFixed(1)},${before.y.toFixed(1)}) ` +
      `-> (${this.state.pose.x.toFixed(1)},${this.state.pose.y.toFixed(1)})`);
    return true;
  }

  private emergencyStopWithFalse(reason: string): false {
    this.emergencyStop(reason);
    return false;
  }

  private sampleRotation(pose: Pose, drivingDisk: Disk, signedAngleDeg: number): Pose[] {
    // The non-driving disk is the temporary pivot; sample intermediate positions.
    const pivotDisk: Disk = drivingDisk === 'TOP' ? 'BOTTOM' : 'TOP';
    const pivot = this.diskCenter(pose, pivotDisk);
    const count = Math.max(1, Math.ceil(Math.abs(signedAngleDeg) / 2));
    const samples: Pose[] = [];
    for (let i = 1; i <= count; i += 1) {
      const angleDeg = pose.angleDeg + signedAngleDeg * i / count;
      const radians = angleDeg * Math.PI / 180;
      const half = this.config.discSpacingCm / 2;
      const pivotOffsetX = pivotDisk === 'TOP' ? half * Math.sin(radians) : -half * Math.sin(radians);
      const pivotOffsetY = pivotDisk === 'TOP' ? -half * Math.cos(radians) : half * Math.cos(radians);
      samples.push({ x: pivot.x - pivotOffsetX, y: pivot.y - pivotOffsetY, angleDeg });
    }
    return samples;
  }

  private diskCenter(pose: Pose, disk: Disk): { x: number; y: number } {
    const half = this.config.discSpacingCm / 2;
    const radians = pose.angleDeg * Math.PI / 180;
    const side = disk === 'TOP' ? 1 : -1;
    return {
      x: pose.x + side * half * Math.sin(radians),
      y: pose.y - side * half * Math.cos(radians),
    };
  }

  private isPoseWithinWindow(pose: Pose): boolean {
    const radius = this.config.discRadiusCm;
    const epsilon = 1e-8;
    for (const disk of ['TOP', 'BOTTOM'] as const) {
      const { x, y } = this.diskCenter(pose, disk);
      if (x - radius < -epsilon || y - radius < -epsilon ||
          x + radius > this.config.widthCm + epsilon ||
          y + radius > this.config.heightCm + epsilon) return false;
    }
    return true;
  }

  private coveragePercent(): number {
    const columns = Math.ceil(this.config.widthCm / this.config.gridCm);
    const rows = Math.ceil(this.config.heightCm / this.config.gridCm);
    return 100 * this.state.cleanedCells.size / (columns * rows);
  }
}

// --- Example setup ---
const windowConfig: WindowConfig = {
  widthCm: 120,
  heightCm: 90,
  discRadiusCm: 6,
  discSpacingCm: 12,
  minimumPressure: 70,
  gridCm: 5,
  horizontalStepAngleDeg: 12,
  verticalStepAngleDeg: 36.25,
  motorSpeedDegPerSec: 45,
};

const robot = new WindowRobotSimulator(windowConfig, { x: 7, y: 12, angleDeg: 0 }, true);
robot.powerOn();
robot.cleanWindow(1);
robot.renderRobot();
robot.renderWindow();

// To test vacuum failure manually, instantiate a fresh robot, power it on,
// start a mode, set state.pressure = 50, then call simulationStep().
