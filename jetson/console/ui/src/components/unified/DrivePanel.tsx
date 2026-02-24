import { Button, Card, Slider, Typography } from "antd";
import { PoweroffOutlined, WarningOutlined } from "@ant-design/icons";
import type { MouseEvent, TouchEvent } from "react";

const { Text } = Typography;

export type DrivePreview = {
  throttle: number;
  turn: number;
  left: number;
  right: number;
};

type DrivePanelProps = {
  wsConnected: boolean;
  spaceHeld: boolean;
  armed: boolean;
  joystickX: number;
  joystickY: number;
  preview: DrivePreview;
  speedLimit: number;
  onJoystickMouseDown: (event: MouseEvent<HTMLDivElement>) => void;
  onJoystickTouchStart: (event: TouchEvent<HTMLDivElement>) => void;
  onArmGate: () => void;
  onStop: () => void;
  onEstop: () => void;
  onSpeedLimitChange: (value: number) => void;
};

function signed(value: number): string {
  const safe = Math.abs(value) < 0.0005 ? 0 : value;
  return `${safe >= 0 ? "+" : ""}${safe.toFixed(2)}`;
}

export function DrivePanel({
  wsConnected,
  spaceHeld,
  armed,
  joystickX,
  joystickY,
  preview,
  speedLimit,
  onJoystickMouseDown,
  onJoystickTouchStart,
  onArmGate,
  onStop,
  onEstop,
  onSpeedLimitChange,
}: DrivePanelProps): JSX.Element {
  const deadmanEnabled = wsConnected && spaceHeld;

  return (
    <Card
      className="hmi-card unified-card"
      title={
        <span className="unified-card-title">
          Drive <span className="muted">(Skid Steer)</span>
        </span>
      }
    >
      <div className={`unified-deadman-row ${deadmanEnabled ? "is-enabled" : "is-disabled"}`}>
        <span className="unified-deadman-check" aria-hidden="true">
          {deadmanEnabled ? "[x]" : "[ ]"}
        </span>
        <span className="unified-deadman-text">Hold SPACE) {deadmanEnabled ? "ENABLED" : "DISABLED"}</span>
        <span className="unified-status-dot" aria-hidden="true" />
      </div>

      <div className="unified-drive-main-row">
        <div
          className="unified-joystick"
          onMouseDown={onJoystickMouseDown}
          onTouchStart={onJoystickTouchStart}
          role="slider"
          aria-label="Virtual joystick"
        >
          <div className="unified-joystick-cross" />
          <div className="unified-joystick-arrow up">^</div>
          <div className="unified-joystick-arrow left">&lt;</div>
          <div className="unified-joystick-arrow right">&gt;</div>
          <div className="unified-joystick-arrow down">v</div>
          <div className="unified-joystick-core" />
          <div
            className="unified-joystick-knob"
            style={{
              transform: `translate(calc(-50% + ${(joystickX * 78).toFixed(1)}px), calc(-50% + ${(-joystickY * 78).toFixed(1)}px))`,
            }}
          >
            <span className="unified-joystick-hand" aria-label="drag">
              drag
            </span>
          </div>
        </div>

        <div className="unified-command-preview">
          <div className="unified-command-preview-title">Command Preview</div>
          <div className="unified-command-row">
            <span>Throttle:</span>
            <code>{signed(preview.throttle)}</code>
          </div>
          <div className="unified-command-row">
            <span>Turn:</span>
            <code>{signed(preview.turn)}</code>
          </div>
          <div className="unified-command-row">
            <span>Left:</span>
            <code>{signed(preview.left)}</code>
          </div>
          <div className="unified-command-row">
            <span>Right:</span>
            <code>{signed(preview.right)}</code>
          </div>
        </div>
      </div>

      <Button className="unified-gate-btn" onClick={onArmGate}>
        Deadman + Safety Gate
      </Button>

      <div className="unified-stop-row">
        <Button className="unified-stop-btn" danger onClick={onStop} icon={<WarningOutlined />}>
          STOP
        </Button>
        <Button className="unified-estop-btn" danger onClick={onEstop} icon={<PoweroffOutlined />}>
          E-STOP
        </Button>
      </div>

      <div className="unified-speed-row">
        <Text className="unified-speed-label">Speed limit {armed ? "" : "(disarmed)"}</Text>
        <Slider min={0.1} max={1.0} step={0.01} value={speedLimit} onChange={(value) => onSpeedLimitChange(Number(value))} />
      </div>
    </Card>
  );
}
