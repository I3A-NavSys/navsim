import { Component, OnDestroy, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Subject, Subscription, of } from 'rxjs';
import {
  catchError,
  debounceTime,
  distinctUntilChanged,
  filter,
  finalize,
  switchMap,
  tap,
} from 'rxjs/operators';

import { CanvasComponent } from './components/canvas.component';
import { InfoPanelComponent } from './components/info-panel.component';
import { DerivativesComponent } from './components/derivatives.component';
import {
  FlightPlan,
  FlightPlanService,
  Trace,
  Waypoint,
} from './services/flight-plan.service';

type Tool = 'add' | 'move' | 'vel';

/**
 * A single mutation request that the AppComponent emits every time the
 * user changes something.  All of them are processed by the same
 * `switchMap` pipeline so:
 *
 *   1. The displayed plan + trace are always coherent.
 *   2. Out-of-order responses can never corrupt the UI.
 *   3. A rapid burst of drag-ticks is collapsed into the latest value
 *      (with a small debounce so the user sees a real-time preview).
 */
interface Mutation {
  kind: 'add' | 'patch' | 'delete' | 'connect' | 'sync' | 'reset';
  payload?: any;
}

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    CanvasComponent,
    InfoPanelComponent,
    DerivativesComponent,
  ],
  template: `
    <div class="app-shell">
      <header class="app-header">
        <h1>FlightPlan Editor</h1>
        <span class="subtitle">
          Angular + Python &middot; drag the waypoint, drag the green
          velocity arrow to rotate/scale it
        </span>
        <span class="status"
              [class.busy]="busy"
              [class.error]="lastError">
          {{ lastError ? '⚠ ' + lastError : (busy ? '… syncing' : '✔ in sync') }}
        </span>
      </header>

      <div class="app-body">
        <aside class="left-pane">
          <app-canvas #canvas
            [waypoints]="waypoints"
            [tracePoints]="tracePoints"
            [tool]="tool"
            [showVelocity]="showVelocity"
            [showTrace]="showTrace"
            [pixelsPerMeter]="pixelsPerMeter"
            [worldOrigin]="worldOrigin"
            (toolChange)="tool = $event"
            (showVelocityChange)="showVelocity = $event"
            (showTraceChange)="showTrace = $event"
            (addWpRequested)="onAddWpRequested($event)"
            (wpMoved)="onWpMoved($event)"
            (velChanged)="onVelChanged($event)"
            (zoomChanged)="onZoomChanged($event)"
            (panChanged)="onPanChanged($event)">
          </app-canvas>

          <div class="canvas-zoom-bar">
            <button (click)="zoomBy(0.8)"  title="Zoom out">−</button>
            <button (click)="zoomReset()" title="Reset view">Fit</button>
            <button (click)="zoomBy(1.25)" title="Zoom in">+</button>
            <span class="zoom-label">{{ pixelsPerMeter | number:'1.1-2' }} px/m</span>
            <span class="spacer"></span>
            <label class="check">
              <input type="checkbox" [(ngModel)]="autoConnect">
              Auto-connect (live trace)
            </label>
          </div>
        </aside>

        <aside class="right-pane">
          <app-info-panel
            [planId]="plan?.id ?? ''"
            [waypoints]="waypoints"
            [startTime]="startTime"
            [finishTime]="finishTime"
            [totalLength]="totalLength"
            [defaultZ]="defaultZ"
            [selectedId]="selectedId"
            (reset)="onReset()"
            (connect)="onConnect()"
            (refreshTrace)="onRefreshTrace()"
            (defaultZChange)="defaultZ = $event"
            (updateWp)="onUpdateWp($event)"
            (deleteWp)="onDeleteWp($event)"
            (select)="selectedId = $event">
          </app-info-panel>

          <app-derivatives
            [trace]="trace"
            [waypoints]="waypoints">
          </app-derivatives>
        </aside>
      </div>
    </div>
  `,
  styles: [`
    :host { display: block; height: 100vh; }
    .app-shell {
      display: flex;
      flex-direction: column;
      height: 100vh;
      background: #14141a;
      color: #ddd;
    }
    .app-header {
      display: flex;
      align-items: baseline;
      gap: 16px;
      padding: 8px 16px;
      background: #1a1a22;
      border-bottom: 1px solid #333;
    }
    .app-header h1 { margin: 0; font-size: 18px; color: #ffcc33; }
    .subtitle { color: #888; font-size: 12px; }
    .spacer { flex: 1; }
    .status {
      margin-left: auto;
      font-size: 11px;
      color: #4caf50;
      padding: 2px 8px;
      border: 1px solid #2c5d2c;
      border-radius: 3px;
    }
    .status.busy  { color: #ffb74d; border-color: #6b4a1c; }
    .status.error { color: #ff6e6e; border-color: #6b2c2c; }
    .app-body {
      flex: 1;
      display: grid;
      grid-template-columns: 1.4fr 1fr;
      gap: 8px;
      padding: 8px;
      overflow: hidden;
    }
    .left-pane, .right-pane {
      display: flex;
      flex-direction: column;
      min-height: 0;
      overflow: auto;
    }
    .canvas-zoom-bar {
      display: flex;
      align-items: center;
      gap: 6px;
      padding: 4px;
      background: #2a2a30;
      border-radius: 4px;
      margin-top: 4px;
    }
    .canvas-zoom-bar button {
      background: #3a3a44;
      color: #ddd;
      border: 1px solid #555;
      padding: 2px 10px;
      border-radius: 3px;
      cursor: pointer;
      font-size: 13px;
      min-width: 32px;
    }
    .canvas-zoom-bar button:hover { background: #4a4a55; }
    .zoom-label { color: #aaa; font-size: 11px; min-width: 70px; }
    .check {
      display: flex;
      align-items: center;
      gap: 4px;
      color: #ccc;
      font-size: 11px;
    }
  `],
})
export class AppComponent implements OnInit, OnDestroy {
  tool: Tool = 'add';
  showVelocity = true;
  showTrace = true;
  defaultZ = 50;
  selectedId: string | null = null;

  /** When true, every edit auto-connects the WPs and refreshes the trace. */
  autoConnect = true;

  plan: FlightPlan | null = null;
  waypoints: Waypoint[] = [];
  trace: Trace | null = null;

  /** Zoom of the canvas in pixels-per-meter. */
  pixelsPerMeter = 6;
  /** World-coordinates origin (bottom-left of the canvas, in metres). */
  worldOrigin: [number, number] = [-20, -20];

  busy = false;
  lastError: string | null = null;

  /** All UI mutations flow through this Subject. The pipeline below
   *  dedups, debounces, and switchMaps them into the single
   *  `applyMutation$` HTTP call. */
  private mutations$ = new Subject<Mutation>();
  private sub = new Subscription();
  private inflight = 0;

  constructor(private fp: FlightPlanService) {}

  ngOnInit(): void {
    // ---- 1. Initial load. ------------------------------------------
    this.sub.add(
      this.fp.get().subscribe({
        next: (p) => this.applyPlan(p),
        error: (e) => (this.lastError = this.fmtError(e)),
      }),
    );

    // ---- 2. Single mutation pipeline. -------------------------------
    //   a.   skip the very first emission so we don't fire on subscribe.
    //   b.   coalesce rapid bursts of identical mutations (drag ticks).
    //   c.   debounce a few ms so the spinner shows a single update.
    //   d.   switchMap cancels any in-flight request when a new one
    //        arrives, which is what fixes the original race condition.
    this.sub.add(
      this.mutations$
        .pipe(
          filter((m) => m !== null),
          debounceTime(20),
          distinctUntilChanged((a, b) => JSON.stringify(a) === JSON.stringify(b)),
          tap(() => {
            this.busy = true;
            this.inflight++;
          }),
          switchMap((m) => this.dispatch(m)),
          finalize(() => {
            this.busy = false;
          }),
        )
        .subscribe({
          next: (p) => this.applyPlan(p),
          error: (e) => {
            this.lastError = this.fmtError(e);
            this.busy = false;
          },
        }),
    );
  }

  ngOnDestroy(): void {
    this.sub.unsubscribe();
  }

  // ---------------------------------------------------------------
  // Mutation dispatch — every UI event turns into a `Mutation` and
  // is fed into the same pipeline.  The `switchMap` then guarantees
  // that only the latest response is applied to the UI.
  // ---------------------------------------------------------------
  private dispatch(m: Mutation) {
    switch (m.kind) {
      case 'add':    return this.fp.addWaypoint(m.payload);
      case 'patch':  return this.fp.updateWaypoint(m.payload);
      case 'delete': return this.fp.deleteWaypoint(m.payload.id);
      case 'connect': return this.fp.connect();
      case 'sync':   return this.fp.sync();
      case 'reset':  return this.fp.reset();
    }
  }

  private applyPlan(p: FlightPlan): void {
    this.plan = p;
    this.waypoints = p.waypoints;
    if (p.trace) this.trace = p.trace;
    this.lastError = null;
  }

  // ---------------------------------------------------------------
  // UI event handlers
  // ---------------------------------------------------------------
  onReset(): void {
    this.mutations$.next({ kind: 'reset' });
    this.selectedId = null;
  }

  onConnect(): void {
    this.mutations$.next({ kind: 'connect' });
  }

  onRefreshTrace(): void {
    this.mutations$.next({ kind: 'sync' });
  }

  onAddWpRequested(ev: { x: number; y: number }): void {
    const t = this.waypoints.length === 0 ? 0 : this.finishTime + 5;
    this.mutations$.next({
      kind: 'add',
      payload: {
        time: t,
        pos:  [ev.x, ev.y, this.defaultZ] as [number, number, number],
        vel:  [0, 0, 0] as [number, number, number],
      },
    });
  }

  onWpMoved(ev: { id: string; pos: [number, number, number] }): void {
    if (!this.autoConnect) {
      // Still keep the local state coherent: store the new pos in the
      // waypoint table so the dot follows the mouse.  The server
      // commit happens when the user releases the mouse.
      this.applyOptimistic(ev.id, { pos: ev.pos });
    }
    this.mutations$.next({
      kind: 'patch',
      payload: { id: ev.id, pos: ev.pos },
    });
  }

  onVelChanged(ev: { id: string; vel: [number, number, number] }): void {
    if (!this.autoConnect) {
      this.applyOptimistic(ev.id, { vel: ev.vel });
    }
    this.mutations$.next({
      kind: 'patch',
      payload: { id: ev.id, vel: ev.vel },
    });
  }

  /** Local-table patch used while a drag is in flight and the user
   *  asked to NOT auto-connect (we still want the dot/arrow to
   *  follow the mouse). */
  private applyOptimistic(
    id: string,
    patch: Partial<Pick<Waypoint, 'pos' | 'vel' | 'time'>>,
  ): void {
    const i = this.waypoints.findIndex((w) => w.id === id);
    if (i < 0) return;
    this.waypoints = this.waypoints.map((w, idx) =>
      idx === i ? { ...w, ...patch } : w,
    );
  }

  onUpdateWp(wp: Waypoint): void {
    this.mutations$.next({
      kind: 'patch',
      payload: {
        id:   wp.id,
        time: wp.time,
        pos:  wp.pos,
        vel:  wp.vel,
      },
    });
  }

  onDeleteWp(id: string): void {
    this.mutations$.next({ kind: 'delete', payload: { id } });
    if (this.selectedId === id) this.selectedId = null;
  }

  // ---------------------------------------------------------------
  // Zoom & pan
  // ---------------------------------------------------------------
  onZoomChanged(ppm: number): void {
    this.pixelsPerMeter = ppm;
  }
  onPanChanged(origin: [number, number]): void {
    this.worldOrigin = origin;
  }
  zoomBy(factor: number): void {
    this.pixelsPerMeter = Math.max(1, Math.min(60, this.pixelsPerMeter * factor));
  }
  zoomReset(): void {
    this.pixelsPerMeter = 6;
    this.worldOrigin    = [-20, -20];
  }

  // ---------------------------------------------------------------
  // Derived getters
  // ---------------------------------------------------------------
  get startTime(): number {
    return this.waypoints.length === 0 ? 0 : this.waypoints[0].time;
  }
  get finishTime(): number {
    return this.waypoints.length === 0
      ? 0
      : this.waypoints[this.waypoints.length - 1].time;
  }
  get totalLength(): number {
    let s = 0;
    for (let i = 1; i < this.waypoints.length; i++) {
      const a = this.waypoints[i - 1].pos;
      const b = this.waypoints[i].pos;
      s += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    }
    return s;
  }
  get tracePoints(): [number, number][] {
    if (!this.trace) return [];
    return this.trace.pos.map((p) => [p[0], p[1]]);
  }

  // ---------------------------------------------------------------
  private fmtError(e: any): string {
    if (!e) return 'unknown error';
    if (typeof e === 'string') return e;
    if (e.error && e.error.error) return e.error.error;
    if (e.message) return e.message;
    return JSON.stringify(e);
  }
}
