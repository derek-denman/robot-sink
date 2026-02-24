import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { MouseEvent as ReactMouseEvent, TouchEvent as ReactTouchEvent } from "react";
import { postApi } from "../../lib/api";
import type { ArmJointConfig, StatusPayload } from "../../types";
import { ArmPanel } from "./ArmPanel";
import { DrivePanel } from "./DrivePanel";
import { KeyboardMapStrip } from "./KeyboardMapStrip";
import { TelemetryColumn } from "./TelemetryColumn";

type UnifiedControlPageProps = {
  status: StatusPayload | null;
  wsConnected: boolean;
};

type KeyState = {
  space: boolean;
  up: boolean;
  down: boolean;
  left: boolean;
  right: boolean;
  shift: boolean;
  ctrl: boolean;
  bracketLeft: boolean;
  bracketRight: boolean;
};

const DEFAULT_KEYS: KeyState = {
  space: false,
  up: false,
  down: false,
  left: false,
  right: false,
  shift: false,
  ctrl: false,
  bracketLeft: false,
  bracketRight: false,
};

const JOINT_LABELS = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist", "J5 Wrist2"];

type DragMeta = {
  centerX: number;
  centerY: number;
  radius: number;
};

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

function isEditableElement(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) {
    return false;
  }
  return ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName) || target.isContentEditable;
}

function fallbackJoint(index: number): ArmJointConfig {
  return {
    name: `j${index + 1}`,
    min: -3.14,
    max: 3.14,
    step: 0.01,
    default: 0,
  };
}

function pickNamedPose(namedPoses: string[], key: "H" | "N" | "B"): string {
  const aliases: Record<"H" | "N" | "B", string[]> = {
    H: ["home", "ready", "pregrasp", "stow"],
    N: ["neutral", "ready", "pregrasp", "stow"],
    B: ["stow", "home", "ready"],
  };

  const lookup = new Map(namedPoses.map((pose) => [pose.toLowerCase(), pose]));
  for (const candidate of aliases[key]) {
    const match = lookup.get(candidate);
    if (match) {
      return match;
    }
  }

  if (key === "H") {
    return "home";
  }
  if (key === "N") {
    return "neutral";
  }
  return "stow";
}

export function UnifiedControlPage({ status, wsConnected }: UnifiedControlPageProps): JSX.Element {
  const [keys, setKeys] = useState<KeyState>(DEFAULT_KEYS);
  const keysRef = useRef<KeyState>(DEFAULT_KEYS);
  const [speedLimit, setSpeedLimit] = useState(0.45);
  const [selectedJointIndex, setSelectedJointIndex] = useState(0);
  const [joystickX, setJoystickX] = useState(0);
  const [joystickY, setJoystickY] = useState(0);
  const [draggingJoystick, setDraggingJoystick] = useState(false);
  const dragMetaRef = useRef<DragMeta | null>(null);

  const [jointValues, setJointValues] = useState<Record<string, number>>({});
  const [gripperValue, setGripperValue] = useState(0);

  const baseTimerRef = useRef<number | null>(null);
  const armJogTimerRef = useRef<number | null>(null);
  const baseActiveRef = useRef(false);
  const baseCommandRef = useRef({ linearX: 0, angularZ: 0 });

  const armJoints = status?.arm?.joints || [];
  const namedPoses = status?.arm?.named_poses || [];
  const armed = Boolean(status?.safety?.armed);

  useEffect(() => {
    if (!armJoints.length) {
      return;
    }

    setJointValues((prev) => {
      const next = { ...prev };
      for (let i = 0; i < Math.min(5, armJoints.length); i += 1) {
        const joint = armJoints[i];
        if (!(joint.name in next)) {
          next[joint.name] = Number(joint.default ?? 0);
        }
      }
      return next;
    });
  }, [armJoints]);

  const updateKey = useCallback((key: keyof KeyState, value: boolean) => {
    if (keysRef.current[key] === value) {
      return;
    }
    const next = { ...keysRef.current, [key]: value };
    keysRef.current = next;
    setKeys(next);
  }, []);

  const resetInputs = useCallback(() => {
    keysRef.current = DEFAULT_KEYS;
    setKeys(DEFAULT_KEYS);
    setJoystickX(0);
    setJoystickY(0);
    setDraggingJoystick(false);
    dragMetaRef.current = null;
  }, []);

  const sendBaseStop = useCallback(async () => {
    await postApi("/api/manual/base", { type: "stop" });
  }, []);

  const sendArmStop = useCallback(async () => {
    await postApi("/api/manual/arm", { action: "stop" });
  }, []);

  const sendGripper = useCallback(async (command: "open" | "close") => {
    await postApi("/api/manual/arm", { gripper: command });
    setGripperValue(command === "open" ? 1 : 0);
  }, []);

  const sendNamedPose = useCallback(
    async (key: "H" | "N" | "B") => {
      const pose = pickNamedPose(namedPoses, key);
      await postApi("/api/manual/arm", { pose });
    },
    [namedPoses],
  );

  const armSafetyGate = useCallback(async () => {
    await postApi("/api/arm", { armed: true });
  }, []);

  const estopNow = useCallback(async () => {
    await postApi("/api/safety/estop", {});
  }, []);

  const applyJoystickPoint = useCallback((clientX: number, clientY: number) => {
    const meta = dragMetaRef.current;
    if (!meta) {
      return;
    }
    const dx = (clientX - meta.centerX) / meta.radius;
    const dy = (meta.centerY - clientY) / meta.radius;
    const mag = Math.hypot(dx, dy);

    if (mag > 1) {
      setJoystickX(dx / mag);
      setJoystickY(dy / mag);
      return;
    }

    setJoystickX(dx);
    setJoystickY(dy);
  }, []);

  const onJoystickMouseDown = useCallback(
    (event: ReactMouseEvent<HTMLDivElement>) => {
      event.preventDefault();
      const rect = event.currentTarget.getBoundingClientRect();
      dragMetaRef.current = {
        centerX: rect.left + rect.width / 2,
        centerY: rect.top + rect.height / 2,
        radius: Math.max(40, Math.min(rect.width, rect.height) * 0.42),
      };
      setDraggingJoystick(true);
      applyJoystickPoint(event.clientX, event.clientY);
    },
    [applyJoystickPoint],
  );

  const onJoystickTouchStart = useCallback(
    (event: ReactTouchEvent<HTMLDivElement>) => {
      if (!event.touches[0]) {
        return;
      }
      const touch = event.touches[0];
      const rect = event.currentTarget.getBoundingClientRect();
      dragMetaRef.current = {
        centerX: rect.left + rect.width / 2,
        centerY: rect.top + rect.height / 2,
        radius: Math.max(40, Math.min(rect.width, rect.height) * 0.42),
      };
      setDraggingJoystick(true);
      applyJoystickPoint(touch.clientX, touch.clientY);
    },
    [applyJoystickPoint],
  );

  useEffect(() => {
    if (!draggingJoystick) {
      return;
    }

    const onMouseMove = (event: MouseEvent) => {
      applyJoystickPoint(event.clientX, event.clientY);
    };

    const onMouseUp = () => {
      setDraggingJoystick(false);
      dragMetaRef.current = null;
      if (!keysRef.current.space) {
        setJoystickX(0);
        setJoystickY(0);
      }
    };

    const onTouchMove = (event: TouchEvent) => {
      if (!event.touches[0]) {
        return;
      }
      applyJoystickPoint(event.touches[0].clientX, event.touches[0].clientY);
    };

    const onTouchEnd = () => {
      onMouseUp();
    };

    window.addEventListener("mousemove", onMouseMove);
    window.addEventListener("mouseup", onMouseUp);
    window.addEventListener("touchmove", onTouchMove, { passive: true });
    window.addEventListener("touchend", onTouchEnd);

    return () => {
      window.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp);
      window.removeEventListener("touchmove", onTouchMove);
      window.removeEventListener("touchend", onTouchEnd);
    };
  }, [applyJoystickPoint, draggingJoystick]);

  const keyboardThrottle = (keys.up ? 1 : 0) + (keys.down ? -1 : 0);
  const keyboardTurn = (keys.left ? 1 : 0) + (keys.right ? -1 : 0);

  const throttle = clamp(keyboardThrottle + joystickY, -1, 1);
  const turn = clamp(keyboardTurn - joystickX, -1, 1);
  const left = clamp(throttle + turn, -1, 1);
  const right = clamp(throttle - turn, -1, 1);

  const speedMultiplier = keys.ctrl ? 0.35 : keys.shift ? 1.45 : 1;
  const commandScale = speedLimit * speedMultiplier;

  const linearX = throttle * commandScale;
  const angularZ = turn * commandScale;

  const shouldMove = Math.abs(throttle) > 0.01 || Math.abs(turn) > 0.01;
  const shouldPublishBase = wsConnected && armed && keys.space && shouldMove;

  baseCommandRef.current = { linearX, angularZ };

  useEffect(() => {
    const tick = async () => {
      const command = baseCommandRef.current;
      await postApi("/api/manual/base", {
        type: "cmd_vel",
        linear_x: command.linearX,
        angular_z: command.angularZ,
      });
    };

    if (shouldPublishBase) {
      baseActiveRef.current = true;
      if (baseTimerRef.current !== null) {
        window.clearInterval(baseTimerRef.current);
      }

      tick().catch(() => undefined);
      baseTimerRef.current = window.setInterval(() => {
        tick().catch(() => undefined);
      }, 110);
      return;
    }

    if (baseTimerRef.current !== null) {
      window.clearInterval(baseTimerRef.current);
      baseTimerRef.current = null;
    }

    if (baseActiveRef.current) {
      baseActiveRef.current = false;
      sendBaseStop().catch(() => undefined);
    }
  }, [sendBaseStop, shouldPublishBase]);

  const sendJog = useCallback(
    async (direction: number) => {
      if (selectedJointIndex === 5) {
        await sendGripper(direction > 0 ? "open" : "close");
        return;
      }

      const joint = armJoints[selectedJointIndex] || fallbackJoint(selectedJointIndex);
      const step = Math.abs(Number(joint.step || 0.01));
      const delta = direction * step;
      await postApi("/api/manual/arm", { joint: joint.name, delta });
    },
    [armJoints, selectedJointIndex, sendGripper],
  );

  const jogDirection = useMemo(() => {
    if (keys.bracketLeft && !keys.bracketRight) {
      return -1;
    }
    if (keys.bracketRight && !keys.bracketLeft) {
      return 1;
    }
    return 0;
  }, [keys.bracketLeft, keys.bracketRight]);

  useEffect(() => {
    if (!wsConnected || jogDirection === 0) {
      if (armJogTimerRef.current !== null) {
        window.clearInterval(armJogTimerRef.current);
        armJogTimerRef.current = null;
      }
      return;
    }

    sendJog(jogDirection).catch(() => undefined);
    armJogTimerRef.current = window.setInterval(() => {
      sendJog(jogDirection).catch(() => undefined);
    }, 120);

    return () => {
      if (armJogTimerRef.current !== null) {
        window.clearInterval(armJogTimerRef.current);
        armJogTimerRef.current = null;
      }
    };
  }, [jogDirection, sendJog, wsConnected]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (isEditableElement(event.target)) {
        return;
      }

      if (event.key === " " || event.key === "Spacebar") {
        event.preventDefault();
        updateKey("space", true);
        return;
      }

      if (event.key === "ArrowUp") {
        event.preventDefault();
        updateKey("up", true);
        return;
      }

      if (event.key === "ArrowDown") {
        event.preventDefault();
        updateKey("down", true);
        return;
      }

      if (event.key === "ArrowLeft") {
        event.preventDefault();
        updateKey("left", true);
        return;
      }

      if (event.key === "ArrowRight") {
        event.preventDefault();
        updateKey("right", true);
        return;
      }

      if (event.key === "Shift") {
        updateKey("shift", true);
        return;
      }

      if (event.key === "Control") {
        updateKey("ctrl", true);
        return;
      }

      if (event.key === "[") {
        event.preventDefault();
        updateKey("bracketLeft", true);
        return;
      }

      if (event.key === "]") {
        event.preventDefault();
        updateKey("bracketRight", true);
        return;
      }

      if (/^[1-6]$/.test(event.key) && !event.repeat) {
        setSelectedJointIndex(Number(event.key) - 1);
        return;
      }

      if ((event.key === "x" || event.key === "X") && !event.repeat) {
        updateKey("up", false);
        updateKey("down", false);
        updateKey("left", false);
        updateKey("right", false);
        sendBaseStop().catch(() => undefined);
        return;
      }

      if ((event.key === "e" || event.key === "E") && !event.repeat) {
        estopNow().catch(() => undefined);
        return;
      }

      if ((event.key === "o" || event.key === "O") && !event.repeat) {
        sendGripper("open").catch(() => undefined);
        return;
      }

      if ((event.key === "p" || event.key === "P") && !event.repeat) {
        sendGripper("close").catch(() => undefined);
        return;
      }

      if ((event.key === "h" || event.key === "H") && !event.repeat) {
        sendNamedPose("H").catch(() => undefined);
        return;
      }

      if ((event.key === "n" || event.key === "N") && !event.repeat) {
        sendNamedPose("N").catch(() => undefined);
        return;
      }

      if ((event.key === "b" || event.key === "B") && !event.repeat) {
        sendNamedPose("B").catch(() => undefined);
        return;
      }

      if (event.key === "Escape" && !event.repeat) {
        sendArmStop().catch(() => undefined);
      }
    };

    const onKeyUp = (event: KeyboardEvent) => {
      if (event.key === " " || event.key === "Spacebar") {
        updateKey("space", false);
        return;
      }
      if (event.key === "ArrowUp") {
        updateKey("up", false);
        return;
      }
      if (event.key === "ArrowDown") {
        updateKey("down", false);
        return;
      }
      if (event.key === "ArrowLeft") {
        updateKey("left", false);
        return;
      }
      if (event.key === "ArrowRight") {
        updateKey("right", false);
        return;
      }
      if (event.key === "Shift") {
        updateKey("shift", false);
        return;
      }
      if (event.key === "Control") {
        updateKey("ctrl", false);
        return;
      }
      if (event.key === "[") {
        updateKey("bracketLeft", false);
        return;
      }
      if (event.key === "]") {
        updateKey("bracketRight", false);
      }
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [estopNow, sendArmStop, sendBaseStop, sendGripper, sendNamedPose, updateKey]);

  const clearAllControlInputs = useCallback(() => {
    resetInputs();
    sendBaseStop().catch(() => undefined);
  }, [resetInputs, sendBaseStop]);

  useEffect(() => {
    const onBlur = () => {
      clearAllControlInputs();
    };

    const onVisibilityChange = () => {
      if (document.hidden) {
        clearAllControlInputs();
      }
    };

    window.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibilityChange);

    return () => {
      window.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibilityChange);
    };
  }, [clearAllControlInputs]);

  useEffect(() => {
    if (wsConnected) {
      return;
    }
    clearAllControlInputs();
  }, [clearAllControlInputs, wsConnected]);

  useEffect(() => {
    return () => {
      if (baseTimerRef.current !== null) {
        window.clearInterval(baseTimerRef.current);
      }
      if (armJogTimerRef.current !== null) {
        window.clearInterval(armJogTimerRef.current);
      }
      sendBaseStop().catch(() => undefined);
    };
  }, [sendBaseStop]);

  const mappedJoints = JOINT_LABELS.map((_, index) => armJoints[index] || fallbackJoint(index));

  const onJointSliderChange = useCallback((jointName: string, value: number) => {
    setJointValues((prev) => ({
      ...prev,
      [jointName]: value,
    }));
  }, []);

  const onJointSliderCommit = useCallback(async (jointName: string, value: number) => {
    await postApi("/api/manual/arm", {
      joints: {
        [jointName]: value,
      },
    });
  }, []);

  const onGripperSliderCommit = useCallback(
    async (value: number) => {
      await sendGripper(value > 0.5 ? "open" : "close");
    },
    [sendGripper],
  );

  return (
    <div className="unified-page">
      <div className="unified-main-grid">
        <DrivePanel
          wsConnected={wsConnected}
          spaceHeld={keys.space}
          armed={armed}
          joystickX={joystickX}
          joystickY={joystickY}
          preview={{ throttle, turn, left, right }}
          speedLimit={speedLimit}
          onJoystickMouseDown={onJoystickMouseDown}
          onJoystickTouchStart={onJoystickTouchStart}
          onArmGate={() => {
            armSafetyGate().catch(() => undefined);
          }}
          onStop={() => {
            sendBaseStop().catch(() => undefined);
          }}
          onEstop={() => {
            estopNow().catch(() => undefined);
          }}
          onSpeedLimitChange={setSpeedLimit}
        />

        <ArmPanel
          armJoints={mappedJoints}
          namedPoses={namedPoses}
          selectedJointIndex={selectedJointIndex}
          jointValues={jointValues}
          gripperValue={gripperValue}
          onSelectJoint={setSelectedJointIndex}
          onJointSliderChange={onJointSliderChange}
          onJointSliderCommit={(jointName, value) => {
            onJointSliderCommit(jointName, value).catch(() => undefined);
          }}
          onGripperSliderChange={setGripperValue}
          onGripperSliderCommit={(value) => {
            onGripperSliderCommit(value).catch(() => undefined);
          }}
          onPoseTrigger={(key) => {
            sendNamedPose(key).catch(() => undefined);
          }}
          onGripperCommand={(command) => {
            sendGripper(command).catch(() => undefined);
          }}
          onStopArm={() => {
            sendArmStop().catch(() => undefined);
          }}
        />

        <TelemetryColumn wsConnected={wsConnected} status={status} />
      </div>

      <KeyboardMapStrip />
    </div>
  );
}
