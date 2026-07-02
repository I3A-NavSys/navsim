import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

export interface Waypoint {
  id: string;
  time: number;
  pos: [number, number, number];
  vel: [number, number, number];
  acel: [number, number, number];
  jerk: [number, number, number];
  snap: [number, number, number];
  crakle: [number, number, number];
}

export interface FlightPlan {
  id: string;
  priority: number;
  radius: number;
  length: number;
  waypoints: Waypoint[];
  /** When this plan came back from a combined endpoint, the server also
   *  attached the freshly computed trace so the client can update both
   *  the table and the chart in a single round-trip. */
  trace?: Trace;
}

export interface Trace {
  shape: number[];
  t: number[];
  pos: [number, number, number][];
  vel: [number, number, number][];
  acel: [number, number, number][];
  jerk: [number, number, number][];
  snap: [number, number, number][];
  crakle: [number, number, number][];
}

@Injectable({ providedIn: 'root' })
export class FlightPlanService {
  constructor(private http: HttpClient) {}

  /** GET the current plan and trace separately.  Used for the first
   *  load of the page. */
  get(): Observable<FlightPlan> {
    return this.http.get<FlightPlan>('/api/flightplan');
  }

  /** Reset the plan to empty. Returns plan + trace. */
  reset(): Observable<FlightPlan> {
    return this.http.post<FlightPlan>('/api/flightplan/reset', {});
  }

  /** Add a waypoint. Returns the updated plan + trace. */
  addWaypoint(payload: {
    id?: string;
    time: number;
    pos: [number, number, number];
    vel: [number, number, number];
  }): Observable<FlightPlan> {
    return this.http.post<FlightPlan>('/api/waypoint', payload);
  }

  /** Update a waypoint (pos / vel / time). Returns the updated plan + trace. */
  updateWaypoint(payload: {
    id: string;
    pos?: [number, number, number];
    vel?: [number, number, number];
    time?: number;
  }): Observable<FlightPlan> {
    return this.http.patch<FlightPlan>('/api/waypoint', payload);
  }

  /** Delete a waypoint. Returns the updated plan + trace. */
  deleteWaypoint(id: string): Observable<FlightPlan> {
    return this.http.request<FlightPlan>(
      'DELETE', '/api/waypoint', { body: { id } },
    );
  }

  /** Re-run connect_waypoints() on the server. Returns plan + trace. */
  connect(): Observable<FlightPlan> {
    return this.http.post<FlightPlan>('/api/connect', {});
  }

  /** Re-fetch the current state. Returns plan + trace. */
  sync(): Observable<FlightPlan> {
    return this.http.post<FlightPlan>('/api/sync', {});
  }

  /** Fetch the standalone trace (used to refresh the chart without
   *  changing the plan).  Falls back to the empty trace when the plan
   *  has fewer than two waypoints. */
  trace(dt = 0.1): Observable<Trace> {
    return this.http.get<Trace>('/api/trace', { params: { dt } as any });
  }
}
