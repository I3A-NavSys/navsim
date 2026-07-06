import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { map } from 'rxjs/operators';

/** A 3-element array used for positions, velocities, etc. */
export type Vec3 = [number, number, number];

/** A waypoint as serialized by the Rust backend. */
export interface Waypoint {
  id: string;
  fly_over: boolean;
  time: number;
  pos: Vec3;
  vel: Vec3;
  acel: Vec3;
  jerk: Vec3;
  snap: Vec3;
  crakle: Vec3;
  heading: [number, number];
}

/** A serialized flight plan together with its latest trace. */
export interface FlightPlan {
  id: string;
  priority: number;
  radius: number;
  max_var_lin_vel: number;
  max_var_ang_vel: number;
  waypoints: Waypoint[];
  length: number;
  start_time: number | null;
  finish_time: number | null;
  trace: Trace;
  visible: boolean;
}

/** Lightweight flight-plan summary used in the plan picker. */
export interface PlanSummary {
  id: string;
  priority: number;
  radius: number;
  length: number;
  start_time: number | null;
  finish_time: number | null;
  visible: boolean;
}

/** Column-split trace returned by `GET /api/plans/{id}/trace`. */
export interface Trace {
  dt: number;
  t: number[];
  pos: Vec3[];
  vel: Vec3[];
  acel: Vec3[];
  jerk: Vec3[];
  snap: Vec3[];
  crakle: Vec3[];
}

/** Combination returned by `GET /api/plans`. */
export interface PlanList {
  plans: PlanSummary[];
  sim_time: number;
}

/** Names of the five kinematic derivatives shown in the chart panel. */
export type Derivative =
  | 'velocity'
  | 'acceleration'
  | 'jerk'
  | 'snap'
  | 'crackle';

export const DERIVATIVE_LABELS: Record<Derivative, string> = {
  velocity:     'Velocity',
  acceleration: 'Acceleration',
  jerk:         'Jerk',
  snap:         'Snap',
  crackle:      'Crackle',
};

/** Short label for a derivative unit that fits the small footer
 *  under each Chart.js axis chart. */
export const DERIVATIVE_UNITS: Record<Derivative, string> = {
  velocity:     'm/s',
  acceleration: 'm/s²',
  jerk:         'm/s³',
  snap:         'm/s⁴',
  crackle:      'm/s⁵',
};

/** Names of the three world axes used in the 3D viewer. */
export type Axis = 'x' | 'y' | 'z';

export const AXIS_LABELS: Record<Axis, string> = {
  x: 'X',
  y: 'Y',
  z: 'Z',
};

/** Indices into a `Vec3` for each axis name. */
export const AXIS_INDEX: Record<Axis, 0 | 1 | 2> = {
  x: 0,
  y: 1,
  z: 2,
};

/** Returns `vec[AXIS_INDEX[axis]]`. */
export function pickAxis(vec: Vec3, axis: Axis): number {
  return vec[AXIS_INDEX[axis]];
}

/** Colour palette used by the UI (must mirror `--accent` etc. from
 *  styles.css for the canvas / chart to stay coherent). With 12
 *  hues the hash-based mapping rarely collides for typical
 *  workloads (≤ 20 plans), and the spread covers the hue circle. */
export const PALETTE = [
  '#7c5cff',  // violet
  '#2dd4bf',  // teal
  '#fbbf24',  // gold
  '#f43f5e',  // rose
  '#5eead4',  // mint
  '#a78bfa',  // lavender
  '#f97316',  // orange
  '#38bdf8',  // sky
  '#ec4899',  // pink
  '#84cc16',  // lime
  '#facc15',  // yellow
  '#22d3ee',  // cyan
] as const;

/**
 * Centralised FlightPlan HTTP service. Every write endpoint returns
 * the updated plan (and the freshly computed trace), so the client
 * can re-render from a single round-trip.
 */
@Injectable({ providedIn: 'root' })
export class FlightPlanService {
  constructor(private http: HttpClient) {}

  // ---- list & get --------------------------------------------------------
  list(): Observable<PlanList> {
    return this.http.get<PlanList>('/api/plans');
  }

  get(planId: string): Observable<FlightPlan> {
    return this.http.get<FlightPlan>(`/api/plans/${encodeURIComponent(planId)}`);
  }

  // ---- plan management ---------------------------------------------------
  create(payload: {
    id?: string;
    priority?: number;
    radius?: number;
    max_var_lin_vel?: number;
    max_var_ang_vel?: number;
    waypoints?: Waypoint[];
  }): Observable<FlightPlan> {
    return this.http.post<FlightPlan>('/api/plans', payload);
  }

  delete(planId: string): Observable<void> {
    return this.http.delete<void>(`/api/plans/${encodeURIComponent(planId)}`);
  }

  updateAttributes(
    planId: string,
    patch: Partial<{
      priority: number;
      radius: number;
      max_var_lin_vel: number;
      max_var_ang_vel: number;
    }>,
  ): Observable<FlightPlan> {
    return this.http.patch<FlightPlan>(
      `/api/plans/${encodeURIComponent(planId)}`,
      patch,
    );
  }

  // ---- waypoint management ----------------------------------------------
  addWaypoint(
    planId: string,
    payload: {
      id?: string;
      time: number;
      pos: Vec3;
      vel?: Vec3;
      fly_over?: boolean;
      heading?: [number, number];
    },
  ): Observable<FlightPlan> {
    return this.http.post<FlightPlan>(
      `/api/plans/${encodeURIComponent(planId)}/waypoints`,
      payload,
    );
  }

  updateWaypoint(
    planId: string,
    payload: {
      id: string;
      time?: number;
      pos?: Vec3;
      vel?: Vec3;
      fly_over?: boolean;
      heading?: [number, number];
    },
  ): Observable<FlightPlan> {
    return this.http.patch<FlightPlan>(
      `/api/plans/${encodeURIComponent(planId)}/waypoints/patch`,
      payload,
    );
  }

  deleteWaypoint(planId: string, id: string): Observable<FlightPlan> {
    return this.http.request<FlightPlan>(
      'DELETE',
      `/api/plans/${encodeURIComponent(planId)}/waypoints/patch`,
      { body: { id } },
    );
  }

  connect(planId: string): Observable<FlightPlan> {
    return this.http.post<FlightPlan>(
      `/api/plans/${encodeURIComponent(planId)}/connect`,
      {},
    );
  }

  // ---- simulation clock --------------------------------------------------
  setSimTime(sim_time: number | null): Observable<number> {
    const body =
      sim_time === null
        ? { reset: true }
        : { reset: false, time: sim_time };
    return this.http
      .post<{ sim_time: number }>('/api/sim', body)
      .pipe(map((r) => r.sim_time));
  }
}
