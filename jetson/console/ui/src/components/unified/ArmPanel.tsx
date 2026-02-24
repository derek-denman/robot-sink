import { Button, Card, Slider } from "antd";
import type { ArmJointConfig } from "../../types";

type ArmPanelProps = {
  armJoints: ArmJointConfig[];
  namedPoses: string[];
  selectedJointIndex: number;
  jointValues: Record<string, number>;
  gripperValue: number;
  onSelectJoint: (index: number) => void;
  onJointSliderChange: (jointName: string, value: number) => void;
  onJointSliderCommit: (jointName: string, value: number) => void;
  onGripperSliderChange: (value: number) => void;
  onGripperSliderCommit: (value: number) => void;
  onPoseTrigger: (poseKey: "H" | "N" | "B") => void;
  onGripperCommand: (command: "open" | "close") => void;
  onStopArm: () => void;
};

const JOINT_LABELS = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist", "J5 Wrist2"];

function fallbackJoint(index: number): ArmJointConfig {
  return {
    name: `j${index + 1}`,
    min: -3.14,
    max: 3.14,
    step: 0.01,
    default: 0,
  };
}

function poseAvailable(namedPoses: string[], pose: string): boolean {
  const lower = pose.toLowerCase();
  return namedPoses.some((item) => item.toLowerCase() === lower);
}

export function ArmPanel({
  armJoints,
  namedPoses,
  selectedJointIndex,
  jointValues,
  gripperValue,
  onSelectJoint,
  onJointSliderChange,
  onJointSliderCommit,
  onGripperSliderChange,
  onGripperSliderCommit,
  onPoseTrigger,
  onGripperCommand,
  onStopArm,
}: ArmPanelProps): JSX.Element {
  const joints = JOINT_LABELS.map((label, index) => ({
    label,
    config: armJoints[index] || fallbackJoint(index),
    keyNumber: index + 1,
  }));

  return (
    <Card
      className="hmi-card unified-card"
      title={
        <span className="unified-card-title">
          Arm <span className="muted">(RoArm-MS-2)</span>
        </span>
      }
    >
      <div className="unified-section-title">Joint Jog</div>

      <div className="unified-joint-list">
        {joints.map((joint, index) => {
          const value = jointValues[joint.config.name] ?? Number(joint.config.default ?? 0);
          return (
            <div className="unified-joint-row" key={joint.label}>
              <div className="unified-joint-label">{joint.label}</div>
              <Slider
                min={joint.config.min}
                max={joint.config.max}
                step={joint.config.step || 0.01}
                value={value}
                onChange={(next) => onJointSliderChange(joint.config.name, Number(next))}
                onChangeComplete={(next) => onJointSliderCommit(joint.config.name, Number(next))}
              />
              <button
                type="button"
                className={`unified-key-pill ${selectedJointIndex === index ? "is-active" : ""}`}
                onClick={() => onSelectJoint(index)}
              >
                {joint.keyNumber}
              </button>
            </div>
          );
        })}

        <div className="unified-joint-row" key="gripper">
          <div className="unified-joint-label">Gripper</div>
          <Slider
            min={0}
            max={1}
            step={1}
            value={gripperValue}
            onChange={(next) => onGripperSliderChange(Number(next))}
            onChangeComplete={(next) => onGripperSliderCommit(Number(next))}
          />
          <button
            type="button"
            className={`unified-key-pill ${selectedJointIndex === 5 ? "is-active" : ""}`}
            onClick={() => onSelectJoint(5)}
          >
            6
          </button>
        </div>
      </div>

      <div className="unified-divider" />

      <div className="unified-section-title">Named Poses + Gripper</div>
      <div className="unified-pose-row">
        <button type="button" className="unified-action-key" onClick={() => onPoseTrigger("H")}>
          <span className="keycap">H</span>
          <span>Home</span>
        </button>
        <button type="button" className="unified-action-key" onClick={() => onPoseTrigger("N")}>
          <span className="keycap">N</span>
          <span>Neutral</span>
        </button>
        <button type="button" className="unified-action-key" onClick={() => onPoseTrigger("B")}>
          <span className="keycap">B</span>
          <span>Stow</span>
        </button>
      </div>

      <div className="unified-pose-row">
        <button type="button" className="unified-action-key" onClick={() => onGripperCommand("open")}>
          <span className="keycap">O</span>
          <span>Open</span>
        </button>
        <button type="button" className="unified-action-key" onClick={() => onGripperCommand("close")}>
          <span className="keycap">P</span>
          <span>Close</span>
        </button>
      </div>

      <div className="unified-pose-hint">
        {poseAvailable(namedPoses, "home") ? "home" : "home alias"} | {poseAvailable(namedPoses, "neutral") ? "neutral" : "neutral alias"} | {poseAvailable(namedPoses, "stow") ? "stow" : "stow alias"}
      </div>

      <Button className="unified-stop-arm-btn" danger onClick={onStopArm}>
        ESC STOP ARM NOW
      </Button>
    </Card>
  );
}
