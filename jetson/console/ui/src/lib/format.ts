export function hz(value: number | null | undefined): string {
  return `${Number(value || 0).toFixed(1)} Hz`;
}

export function secAge(value: number | null | undefined): string {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return "n/a";
  }
  return `${Number(value).toFixed(2)} s`;
}

export function agoUnix(unix: number | null | undefined): string {
  if (!unix) {
    return "n/a";
  }
  const age = Math.max(0, Date.now() / 1000 - unix);
  return `${age.toFixed(1)} s ago`;
}

export function bytes(value: number): string {
  if (!Number.isFinite(value) || value <= 0) {
    return "0 B";
  }
  const units = ["B", "KB", "MB", "GB", "TB"];
  let n = value;
  let idx = 0;
  while (n >= 1024 && idx < units.length - 1) {
    n /= 1024;
    idx += 1;
  }
  return `${n.toFixed(idx === 0 ? 0 : 1)} ${units[idx]}`;
}

export function timestampToLocal(unix: number | null | undefined): string {
  if (!unix) {
    return "n/a";
  }
  return new Date(unix * 1000).toLocaleString();
}
