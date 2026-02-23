import type { ApiResult, StatusPayload } from "../types";

const FRIENDLY_ERRORS: Record<string, string> = {
  manual_mode_required: "Switch to Manual mode before sending motion commands.",
  robot_not_armed: "Robot is disarmed. Arm motion first.",
  estop_latched: "E-stop is latched. Reset E-stop before arming.",
  navigate_to_pose_server_unavailable: "Nav2 NavigateToPose action server is unavailable.",
  nav2_msgs_not_available: "nav2_msgs is not available in this ROS environment.",
  no_active_goal: "No active navigation goal.",
  service_unavailable: "Required ROS service is unavailable.",
  slam_save_service_unavailable: "SLAM save-map service is unavailable.",
  switch_to_localization_cmd_not_configured: "Localization switch command is not configured.",
  command_not_configured: "Command is not configured."
};

function friendlyText(value: string): string {
  return value.split("_").join(" ");
}

function extractError(payload: Record<string, unknown>): string {
  const direct = typeof payload.error === "string" ? payload.error : "";
  if (direct) {
    return FRIENDLY_ERRORS[direct] || friendlyText(direct);
  }

  const result = payload.result;
  if (result && typeof result === "object") {
    const nested = result as Record<string, unknown>;
    const nestedError = typeof nested.error === "string" ? nested.error : "";
    if (nestedError) {
      return FRIENDLY_ERRORS[nestedError] || friendlyText(nestedError);
    }

    const nestedResults = Array.isArray(nested.results) ? nested.results : [];
    const firstError = nestedResults.find((entry) => entry && typeof entry === "object" && "error" in (entry as Record<string, unknown>));
    if (firstError && typeof (firstError as Record<string, unknown>).error === "string") {
      const value = (firstError as Record<string, unknown>).error as string;
      return FRIENDLY_ERRORS[value] || friendlyText(value);
    }
  }

  return "Request failed";
}

async function request<T = Record<string, unknown>>(
  path: string,
  init?: RequestInit,
): Promise<T> {
  const response = await fetch(path, {
    ...init,
    headers: {
      Accept: "application/json",
      ...(init?.headers || {})
    }
  });

  let payload: Record<string, unknown> = {};
  try {
    payload = (await response.json()) as Record<string, unknown>;
  } catch {
    payload = {};
  }

  if (!response.ok || payload.ok === false) {
    throw new Error(extractError(payload));
  }

  return payload as T;
}

export async function getStatus(): Promise<StatusPayload> {
  return request<StatusPayload>("/api/status");
}

export async function getConfig<T = Record<string, unknown>>(): Promise<T> {
  return request<T>("/api/config");
}

export async function getStreams<T = Record<string, unknown>>(): Promise<T> {
  return request<T>("/api/streams");
}

export async function postApi<T = ApiResult>(path: string, body?: Record<string, unknown>): Promise<T> {
  return request<T>(path, {
    method: "POST",
    headers: {
      "Content-Type": "application/json"
    },
    body: body ? JSON.stringify(body) : undefined
  });
}

export async function getApi<T = Record<string, unknown>>(path: string): Promise<T> {
  return request<T>(path);
}
