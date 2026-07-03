import {
  ChangeDetectionStrategy,
  Component,
  ElementRef,
  NgZone,
  OnDestroy,
  OnInit,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

import { Observable, Subscription } from 'rxjs';

import { Viewer3dComponent } from './components/viewer3d.component';
import { InfoPanelComponent } from './components/info-panel.component';
import { DerivativesComponent } from './components/derivatives.component';
import { TimelineComponent } from './components/timeline.component';

import {
  FlightPlan,
  FlightPlanService,
  PlanSummary,
  Trace,
  Vec3,
  Waypoint,
} from './services/flight-plan.service';
import { PlanColorService } from './services/plan-color.service';
import {
  MutationStreamService,
  isPlanDeleted,
  MutationResult,
  PlanDeletedEvent,
} from './services/mutation-stream.service';

/** State held in the layout grid for a draggable divider. */
interface Divider {
  /** Direction the splitter adjusts. */
  axis: 'h' | 'v';
  /** Flex-grow of the panel left / above the splitter. */
  left: number;
  /** Flex-grow of the panel right / below the splitter. */
  right: number;
}

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    Viewer3dComponent,
    InfoPanelComponent,
    DerivativesComponent,
    TimelineComponent,
  ],
  // Root component — needs default change detection because data
  // arrives via Observables (cd runs once per Angular zone tick).
  changeDetection: ChangeDetectionStrategy.Default,
  template: `
    <div class="app-shell">
      <header class="topbar">
        <div class="brand">
          <span class="brand-mark"></span>
          <span class="brand-text">FlightPlan Editor</span>
          <span class="version">v0.1 · 3D · multi-plan · live trace</span>
        </div>
        <div class="topbar-mid">
          <div class="plan-tabs">
            <span class="tab"
                    *ngFor="let p of planSummaries; let i = index"
                    [class.active]="currentPlanId === p.id"
                    [style.--c]="planColor(p.id)"
                    [hidden]="!p.visible"
                    (click)="setCurrent(p.id)">
              <span class="dot" [style.background]="planColor(p.id)"></span>
              {{ p.id }}
              <button class="tab-x"
                      title="Delete plan"
                      (click)="deletePlan(p.id, $event)">×</button>
            </span>
            <button class="tab add" (click)="openNewPlanDialog()">
              <span class="plus">＋</span> New plan
            </button>
          </div>
        </div>
        <div class="status">
          <span class="chip"
                [class.busy]="(busy$ | async)"
                [class.error]="(error$ | async)">
            {{ (error$ | async) ? '⚠ ' + (error$ | async)
               : ((busy$ | async) ? '… syncing' : '✔ in sync') }}
          </span>
        </div>
      </header>

      <!-- "New plan" popup — opens when the user clicks "New plan". -->
      <div class="modal-backdrop"
           *ngIf="newPlanDialog"
           (click)="cancelNewPlan($event)">
        <div class="modal" (click)="$event.stopPropagation()">
          <header class="modal-head">
            <h3>New FlightPlan</h3>
            <button class="btn ghost" (click)="cancelNewPlan($event)">×</button>
          </header>
          <div class="modal-body">
            <label>ID
              <input class="input"
                     type="text"
                     [ngModel]="newPlanForm.id"
                     (ngModelChange)="newPlanForm.id = $event" />
            </label>
            <label>priority
              <input class="input"
                     type="number"
                     [ngModel]="newPlanForm.priority"
                     (ngModelChange)="newPlanForm.priority = +$event" />
            </label>
            <label>radius (m)
              <input class="input"
                     type="number"
                     step="0.1"
                     [ngModel]="newPlanForm.radius"
                     (ngModelChange)="newPlanForm.radius = +$event" />
            </label>
            <label>max var lin vel (m/s)
              <input class="input"
                     type="number"
                     step="0.1"
                     [ngModel]="newPlanForm.max_var_lin_vel"
                     (ngModelChange)="newPlanForm.max_var_lin_vel = +$event" />
            </label>
            <label>max var ang vel (rad/s)
              <input class="input"
                     type="number"
                     step="0.1"
                     [ngModel]="newPlanForm.max_var_ang_vel"
                     (ngModelChange)="newPlanForm.max_var_ang_vel = +$event" />
            </label>
          </div>
          <footer class="modal-foot">
            <button class="btn ghost" (click)="cancelNewPlan($event)">Cancel</button>
            <button class="btn primary"
                    [disabled]="!newPlanForm.id"
                    (click)="confirmNewPlan()">Create</button>
          </footer>
        </div>
      </div>

      <!-- ---------- main resizable area ---------- -->
      <main class="layout"
            [style.grid-template-columns]="gridCols()"
            [style.grid-template-rows]="gridRows()">

        <!-- LEFT-TOP: 3D viewer (Z-up convention). The viewer owns
             its own toolbar (tool select, default X/Y/Z, axes, grid)
             so the surrounding shell stays uncluttered. -->
        <section class="cell viewer-cell"
                 [style.grid-column]="'1 / 2'"
                 [style.grid-row]="'1 / 2'">
          <app-viewer3d
            [plans]="plans"
            [simPoints]="simPoints"
            [defaultX]="defaultX"
            [defaultY]="defaultY"
            [defaultZ]="defaultZ"
            (addWpRequested)="onAddWpRequested($event)"
            (wpMoved)="onWpMoved($event)"
            (velChanged)="onVelChanged($event)">
          </app-viewer3d>
        </section>

        <!-- RIGHT-TOP: info + derivatives stacked. The grid has
             three columns (left / 6px divider / right); the info
             panel lives in column 3, spanning the full first row
             of the layout grid. -->
        <section class="cell info-cell"
                 [style.grid-column]="'3 / 4'"
                 [style.grid-row]="'1 / 2'">
          <div class="info-stack">
            <div class="info-card" [style.flex]="infoTopFlex()">
              <app-info-panel
                [plan]="currentPlan"
                [selectedId]="selectedId"
                [color]="currentPlanColor()"
                (select)="selectedId = $event"
                (addWaypoint)="onAddViaButton()"
                (connect)="onConnect()"
                (deleteWp)="onDeleteWp($event)"
                (updateWp)="onUpdateWp($event)"
                (updateAttr)="onUpdateAttr($event)"
                (renameId)="onRenameId($event)">
              </app-info-panel>
            </div>

            <div class="divider divider-h"
                 (mousedown)="startDrag($event, 'infoStack')"
                 title="Drag to resize info / derivatives"></div>

            <div class="deriv-card" [style.flex]="infoBottomFlex()">
              <app-derivatives
                [plans]="plans">
              </app-derivatives>
            </div>
          </div>
        </section>

        <!-- vertical splitter between LEFT and RIGHT columns -->
        <div class="divider divider-v"
             (mousedown)="startDrag($event, 'main')"
             title="Drag to resize viewer / info"></div>

        <!-- hidden full-width bottom row: just for grid math -->
        <section class="cell sim-band"
                 [style.grid-column]="'1 / 4'"
                 [style.grid-row]="'2 / 3'">
          <app-timeline
            [plans]="plans"
            [simTime]="simTime"
            [tMax]="tMax"
            [running]="simRunning"
            [speed]="simSpeed"
            (simTimeChange)="onScrubTime($event)"
            (reset)="onResetSimulation()"
            (playToggle)="toggleSim()"
            (speedChange)="simSpeed = $event">
          </app-timeline>
        </section>
      </main>
    </div>
  `,
  styles: [`
    :host { display: block; height: 100vh; }
    .app-shell {
      display: flex;
      flex-direction: column;
      height: 100vh;
      background: var(--bg-0);
      color: var(--fg-1);
    }
    .topbar {
      display: grid;
      grid-template-columns: 1fr 2fr 1fr;
      align-items: center;
      padding: 8px 14px;
      background: var(--bg-1);
      border-bottom: 1px solid var(--border-soft);
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 10px;
      font-weight: 600;
    }
    .brand-mark {
      width: 22px; height: 22px;
      border-radius: 6px;
      background: linear-gradient(135deg, var(--accent), var(--teal));
      box-shadow: 0 0 16px var(--accent-soft);
    }
    .brand-text { color: var(--fg-0); letter-spacing: -0.01em; }
    .version { color: var(--fg-2); font-size: 11px; }

    .plan-tabs {
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
      justify-content: center;
    }
    .tab {
      background: var(--bg-2);
      border: 1px solid var(--border);
      color: var(--fg-1);
      padding: 4px 6px 4px 8px;
      border-radius: 999px;
      cursor: pointer;
      font-size: 12px;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      position: relative;
      transition: all var(--t-fast);
      user-select: none;
    }
    .tab:hover { background: var(--bg-3); }
    .tab.active {
      background: color-mix(in srgb, var(--c, var(--accent)) 22%, var(--bg-1));
      border-color: color-mix(in srgb, var(--c, var(--accent)) 45%, transparent);
      color: color-mix(in srgb, var(--c, var(--accent)) 75%, white);
    }
    .tab.add {
      background: transparent;
      border-style: dashed;
      color: var(--fg-2);
    }
    .tab-x {
      background: transparent;
      border: none;
      color: var(--fg-2);
      cursor: pointer;
      padding: 0 4px;
      margin-left: 2px;
      border-radius: 4px;
      font-size: 14px;
      line-height: 1;
    }
    .tab-x:hover { background: rgba(244, 63, 94, 0.18); color: var(--danger); }

    .status {
      display: flex;
      justify-content: flex-end;
      align-items: center;
      gap: 8px;
    }
    .chip.busy {
      color: var(--warn);
      border-color: rgba(245, 158, 11, 0.35);
      background: rgba(245, 158, 11, 0.10);
    }
    .chip.error {
      color: var(--danger);
      border-color: rgba(244, 63, 94, 0.35);
      background: rgba(244, 63, 94, 0.10);
    }

    main.layout {
      flex: 1;
      display: grid;
      gap: 0;
      padding: 10px;
      min-height: 0;
    }

    .cell {
      background: transparent;
      padding: 0;
      min-width: 0;
      min-height: 0;
      display: flex;
      flex-direction: column;
    }
    .info-cell  { padding: 0 0 0 8px; min-height: 0; }
    .sim-band   { padding: 8px 0 0 0; }

    /* Right-hand stack: a flex column split by a draggable divider
     * into an info-card (top) and a derivatives-card (bottom). Using
     * flex here (instead of grid) gives us well-defined sizes that
     * do not collapse when the content of either card is empty. */
    .info-stack {
      display: flex;
      flex-direction: column;
      gap: 6px;
      height: 100%;
      min-height: 0;
    }
    .info-card, .deriv-card {
      flex: 1 1 0;
      min-height: 0;
      overflow: hidden;
      background: var(--bg-1);
      border: 1px solid var(--border-soft);
      border-radius: var(--radius);
      display: flex;
      flex-direction: column;
    }
    /* Force Angular host elements to fill the card slot. */
    :host ::ng-deep .info-card > app-info-panel,
    :host ::ng-deep .deriv-card > app-derivatives {
      display: block;
      flex: 1;
      min-height: 0;
    }

    .divider {
      background: var(--bg-2);
      border-radius: 6px;
      transition: background var(--t-fast);
      cursor: col-resize;
    }
    .divider-h {
      cursor: row-resize;
      height: 6px;
    }
    .divider-v {
      width: 6px;
    }
    .divider:hover, .divider.dragging {
      background: linear-gradient(180deg, var(--accent), var(--teal));
    }

    /* Modal dialog */
    .modal-backdrop {
      position: fixed;
      inset: 0;
      background: rgba(0, 0, 0, 0.6);
      backdrop-filter: blur(6px);
      display: flex;
      align-items: center;
      justify-content: center;
      z-index: 100;
    }
    .modal {
      width: 380px;
      max-width: 90vw;
      background: var(--bg-1);
      border: 1px solid var(--border-soft);
      border-radius: var(--radius-lg);
      box-shadow: var(--shadow-3);
      padding: 18px;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }
    .modal h3 { margin: 0; color: var(--fg-0); font-size: 16px; font-weight: 600; }
    .modal-head {
      display: flex; align-items: center; justify-content: space-between;
      border-bottom: 1px solid var(--border-soft); padding-bottom: 8px;
    }
    .modal-body { display: flex; flex-direction: column; gap: 10px; }
    .modal-body label {
      display: grid;
      grid-template-columns: 1fr 1fr;
      align-items: center;
      gap: 8px;
      font-size: 12px;
      color: var(--fg-1);
    }
    .modal-foot {
      display: flex; justify-content: flex-end; gap: 8px;
      border-top: 1px solid var(--border-soft); padding-top: 10px;
    }
  `],
})
export class AppComponent implements OnInit, OnDestroy {
  // ---- inputs from server ----
  plans: FlightPlan[] = [];
  planSummaries: PlanSummary[] = [];
  currentPlanId: string | null = null;
  currentPlan: FlightPlan | null = null;
  selectedId: string | null = null;

  // ---- simulation state ----
  simTime = 0;
  simRunning = false;
  simSpeed = 1;
  simPoints: Map<string, Vec3> = new Map();

  /** Default world coordinates used when the user adds a waypoint
   *  by clicking the 3D viewer. The `default[X/Y/Z]` only takes
   *  effect when the camera is in an axis view that aligns with
   *  that axis (e.g. in the +Y/−Y view, `defaultY` is the locked
   *  altitude of the new waypoint). */
  defaultX = 0;
  defaultY = 0;
  defaultZ = 50;

  /** When true, the simulation loops back to t=0 after reaching
   *  `tMax`. When false, it stops at `tMax` until the user clicks
   *  Reset. Default is false so the simulation reads as smooth and
   *  never jumps back to the start. */
  simLoop = false;

  // ---- layout sizes (flex factors used in the styles) ----
  mainLeft  = 1.4;
  mainRight = 1.0;
  infoTop    = 1.2;
  infoBottom = 1.0;

  // ---- drag state ----
  private dragging: { axis: 'h' | 'v'; startPos: number; startLeft: number; startRight: number; key: string } | null = null;

  // ---- subscriptions ----
  private sub = new Subscription();
  private rafId = 0;

  /** Re-exported for the template. */
  busy$!:  Observable<boolean>;
  error$!: Observable<string | null>;

  constructor(
    private fp: FlightPlanService,
    private stream: MutationStreamService,
    private colors: PlanColorService,
    private zone: NgZone,
    private host: ElementRef<HTMLElement>,
  ) {
    // hot-pull the observables for the template.
    this.busy$  = this.stream.busy$;
    this.error$ = this.stream.error$;
  }

  ngOnInit(): void {
    // Push every mutation response into the plan list.
    this.sub.add(
      this.stream.mutate$.subscribe((p) => this.onMutationResult(p)),
    );

    // 1. Load the initial list of plans.
    this.sub.add(
      this.fp.list().subscribe((l) => {
        this.planSummaries = l.plans;
        if (!this.currentPlanId && this.planSummaries.length) {
          this.setCurrent(this.planSummaries[0].id);
        }
        for (const s of l.plans) {
          this.sub.add(
            this.fp.get(s.id).subscribe((p) => this.replacePlan(p)),
          );
        }
        // Initial sim clock
        this.simTime = l.sim_time;
        this.startSim();
      }),
    );
  }

  ngOnDestroy(): void {
    this.sub.unsubscribe();
    cancelAnimationFrame(this.rafId);
  }

  // ----------------------------------------------------------------
  //  Layout / sizing
  // ----------------------------------------------------------------
  gridCols(): string {
    return `${this.mainLeft}fr 6px ${this.mainRight}fr`;
  }
  gridRows(): string {
    return `1fr 56px`;
  }
  // Flex-grow for the upper / lower cards inside `.info-stack`.
  infoTopFlex(): string    { return `${this.infoTop} 1 0`; }
  infoBottomFlex(): string { return `${this.infoBottom} 1 0`; }

  startDrag(
    ev: MouseEvent,
    which: 'main' | 'infoStack',
  ): void {
    ev.preventDefault();
    document.body.style.cursor = which === 'main' ? 'col-resize' : 'row-resize';
    const target = ev.currentTarget as HTMLElement;
    target.classList.add('dragging');
    const handleMove = (mv: MouseEvent) => {
      // Direct cursor-anchored math: the divider snaps to the cursor,
      // so it never drifts by the click offset on the first move.
      if (which === 'main') {
        const parent = (target.parentElement as HTMLElement).getBoundingClientRect();
        const minPx  = 120;
        const cursorX = mv.clientX - parent.left;
        const leftPx  = Math.max(minPx, Math.min(parent.width - 6 - minPx, cursorX));
        const rightPx = parent.width - 6 - leftPx;
        this.mainLeft  = +(leftPx  / (leftPx + rightPx) * (this.mainLeft + this.mainRight)).toFixed(3);
        this.mainRight = +(rightPx / (leftPx + rightPx) * (this.mainLeft + this.mainRight)).toFixed(3);
      } else {
        const parent = (target.parentElement as HTMLElement).getBoundingClientRect();
        const minPx  = 80;
        const cursorY = mv.clientY - parent.top;
        const topPx  = Math.max(minPx, Math.min(parent.height - 6 - minPx, cursorY));
        const botPx  = parent.height - 6 - topPx;
        this.infoTop    = +(topPx / (topPx + botPx) * (this.infoTop + this.infoBottom)).toFixed(3);
        this.infoBottom = +(botPx / (topPx + botPx) * (this.infoTop + this.infoBottom)).toFixed(3);
      }
    };
    const handleUp = () => {
      document.removeEventListener('mousemove', handleMove);
      document.removeEventListener('mouseup', handleUp);
      document.body.style.cursor = '';
      target.classList.remove('dragging');
    };
    document.addEventListener('mousemove', handleMove);
    document.addEventListener('mouseup', handleUp);
  }

  // ----------------------------------------------------------------
  //  Plan management
  // ----------------------------------------------------------------
  setCurrent(id: string): void {
    this.currentPlanId = id;
    this.currentPlan = this.plans.find((p) => p.id === id) ?? null;
    this.selectedId = null;
  }

  /** Surface the shared colour service to the template. */
  planColor(id: string): string {
    return this.colors.colorFor(id);
  }

  currentPlanColor(): string {
    return this.currentPlanId ? this.colors.colorFor(this.currentPlanId) : '#7c5cff';
  }

  replacePlan(p: FlightPlan): void {
    if (p.id === 'sim' || p.id === '__error__') return;
    const i = this.plans.findIndex((pp) => pp.id === p.id);
    if (i < 0) {
      this.plans = [...this.plans, p];
      // Newly arrived plan → register a placeholder summary so the tab
      // appears immediately (the summary will be refreshed on the
      // next /api/plans fetch).
      if (!this.planSummaries.some((s) => s.id === p.id)) {
        this.planSummaries = [
          ...this.planSummaries,
          {
            id: p.id,
            priority: p.priority,
            radius: p.radius,
            length: p.length,
            start_time: p.start_time,
            finish_time: p.finish_time,
            visible: p.visible,
          },
        ];
      }
    } else {
      const arr = [...this.plans];
      arr[i] = p;
      this.plans = arr;
    }
    if (p.id === this.currentPlanId) this.currentPlan = p;

    // sync tMax for the timeline
    let max = 0;
    for (const pp of this.plans) {
      if (pp.visible && pp.finish_time != null) max = Math.max(max, pp.finish_time!);
    }
    this.tMax = max;
  }

  /** Drop a plan from `plans` / `planSummaries` after the server
   *  confirms a delete. */
  removePlan(planId: string): void {
    this.plans = this.plans.filter((p) => p.id !== planId);
    this.planSummaries = this.planSummaries.filter((s) => s.id !== planId);
    // Drop the simulation dot + colour slot for the deleted plan.
    if (this.simPoints.has(planId)) {
      const next = new Map(this.simPoints);
      next.delete(planId);
      this.simPoints = next;
    }
    this.colors.forget(planId);
    if (this.currentPlanId === planId) {
      this.currentPlanId = null;
      this.currentPlan = null;
      // Pick the first remaining visible plan (if any).
      const next = this.planSummaries.find((s) => s.visible);
      if (next) this.setCurrent(next.id);
    }
    // Refresh tMax.
    let max = 0;
    for (const pp of this.plans) {
      if (pp.visible && pp.finish_time != null) max = Math.max(max, pp.finish_time!);
    }
    this.tMax = max;
  }

  onMutationResult(r: MutationResult): void {
    if (isPlanDeleted(r)) {
      this.removePlan(r.planId);
    } else {
      this.replacePlan(r);
    }
  }

  onAddViaButton(): void {
    if (!this.currentPlanId) return;
    const last = this.currentPlan?.waypoints[this.currentPlan.waypoints.length - 1];
    const t = last ? last.time + 5 : 0;
    const pos: Vec3 = last ? [...last.pos] : [0, 0, this.defaultZ];
    this.stream.push({
      kind: 'add',
      planId: this.currentPlanId,
      payload: { time: t, pos, vel: [0, 0, 0] },
    });
  }

  onAddWpRequested(p: { planId?: string; x: number; y: number; z: number }): void {
    // The clicked plan (if any) wins; if not, fall back to the
    // current plan. If neither exists, we don't add.
    const targetId = p.planId ?? this.currentPlanId;
    if (!targetId) return;
    const target = this.plans.find((pl) => pl.id === targetId);
    const last = target?.waypoints[target.waypoints.length - 1];
    const t = last ? last.time + 5 : 0;
    this.stream.push({
      kind: 'add',
      planId: targetId,
      payload: {
        time: t,
        pos: [+p.x.toFixed(2), +p.y.toFixed(2), +p.z.toFixed(2)] as Vec3,
        vel: [0, 0, 0],
      },
    });
  }

  onWpMoved(ev: { planId: string; id: string; pos: Vec3 }): void {
    if (!ev.planId) return;
    this.stream.push({
      kind: 'patch',
      planId: ev.planId,
      payload: { id: ev.id, pos: ev.pos },
    });
  }

  onVelChanged(ev: { planId: string; id: string; vel: Vec3 }): void {
    if (!ev.planId) return;
    this.stream.push({
      kind: 'patch',
      planId: ev.planId,
      payload: { id: ev.id, vel: ev.vel },
    });
  }

  onDeleteWp(id: string): void {
    if (!this.currentPlanId) return;
    this.stream.push({ kind: 'delete', planId: this.currentPlanId, id });
    if (this.selectedId === id) this.selectedId = null;
  }

  onUpdateWp(wp: Waypoint): void {
    if (!this.currentPlanId) return;
    this.stream.push({
      kind: 'patch',
      planId: this.currentPlanId,
      payload: {
        id: wp.id,
        time: wp.time,
        pos: wp.pos,
        vel: wp.vel,
      },
    });
  }

  onUpdateAttr(ev: { attr: string; value: any }): void {
    if (!this.currentPlanId) return;
    this.stream.push({
      kind: 'planAttr',
      planId: this.currentPlanId,
      payload: { [ev.attr]: ev.value } as any,
    });
  }

  onConnect(): void {
    if (!this.currentPlanId) return;
    this.stream.push({ kind: 'connect', planId: this.currentPlanId });
  }

  onRenameId(newId: string): void {
    // Reuse the planAttr endpoint to set `id` (server keeps the same
    // plan; not ideal but the simplest way without a dedicated rename
    // endpoint). For now, we leave the rename as a no-op in the UI.
    // Future: add POST /api/plans/{id}/rename.
    void newId;
  }

  // ----------------------------------------------------------------
  //  New-plan popup
  // ----------------------------------------------------------------
  /** `true` while the new-plan modal is visible. */
  newPlanDialog = false;
  /** Form values for the new-plan modal — pre-filled with sane defaults. */
  newPlanForm = {
    id: 'Plan',
    priority: 0,
    radius: 1.0,
    max_var_lin_vel: 5.0,
    max_var_ang_vel: 1.0,
  };

  openNewPlanDialog(): void {
    // Suggest an ID that doesn't collide with any existing plan.
    const taken = new Set(this.planSummaries.map((s) => s.id));
    let candidate = 'Plan';
    let n = 1;
    while (taken.has(candidate) || !candidate) {
      n += 1;
      candidate = `Plan${n}`;
    }
    this.newPlanForm = {
      id: candidate,
      priority: 0,
      radius: 1.0,
      max_var_lin_vel: 5.0,
      max_var_ang_vel: 1.0,
    };
    this.newPlanDialog = true;
  }

  cancelNewPlan(ev?: Event): void {
    ev?.stopPropagation?.();
    this.newPlanDialog = false;
  }

  confirmNewPlan(): void {
    // Validate before sending.
    const id = (this.newPlanForm.id || '').trim();
    if (!id) return;
    if (this.planSummaries.some((s) => s.id === id)) {
      // Refuse to silently overwrite an existing plan.
      window.alert(`A plan named '${id}' already exists.`);
      return;
    }
    const payload = {
      id,
      priority: this.newPlanForm.priority | 0,
      radius: Number(this.newPlanForm.radius) || 1,
      max_var_lin_vel: Number(this.newPlanForm.max_var_lin_vel) || 5,
      max_var_ang_vel: Number(this.newPlanForm.max_var_ang_vel) || 1,
    };
    this.stream.push({ kind: 'planCreate', payload });
    this.newPlanDialog = false;
    // Auto-select once the create response arrives.
    setTimeout(() => {
      this.fp.list().subscribe((l) => {
        this.planSummaries = l.plans;
        if (l.plans.some((p) => p.id === id)) this.setCurrent(id);
      });
    }, 200);
  }

  deletePlan(id: string, ev: MouseEvent): void {
    ev.stopPropagation();
    if (!confirm(`Delete plan '${id}'? This cannot be undone.`)) return;
    this.stream.push({ kind: 'planDelete', planId: id });
    if (this.currentPlanId === id) this.currentPlanId = null;
  }

  // ----------------------------------------------------------------
  //  Simulation
  // ----------------------------------------------------------------
  tMax = 60;

  toggleSim(): void {
    this.simRunning = !this.simRunning;
    this.startSim();
  }

  onScrubTime(t: number): void {
    this.simTime = t;
    this.pushSimTime(t);
    this.updateSimPoint();
  }

  onResetSimulation(): void {
    this.simTime = 0;
    this.pushSimTime(null); // null → reset
    this.updateSimPoint();
  }

  private startSim(): void {
    cancelAnimationFrame(this.rafId);
    let last = performance.now();
    const tick = () => {
      if (!this.simRunning) return;
      const now = performance.now();
      const dt = (now - last) / 1000;
      last = now;
      this.simTime += dt * this.simSpeed;
      if (this.simTime > this.tMax) {
        if (this.simLoop) this.simTime = 0;
        else {
          // Pause at the end of the trajectory, do not jump back.
          this.simTime = this.tMax;
          this.simRunning = false;
        }
      }
      this.updateSimPoint();
      this.rafId = requestAnimationFrame(tick);
    };
    this.rafId = requestAnimationFrame(tick);
  }

  /** Interpolate every visible plan at the current sim time and
   *  push the (x, y, z) of every plan's interpolated waypoint to
   *  the viewer.
   *
   *  Position is read directly from the trace — the Rust backend
   *  already runs the 5th-order Taylor expansion when it samples
   *  the flight plan, so the trace.pos array IS the C⁵ path. With
   *  `dt=0.02` (= 50 samples per simulated second) every visible
   *  motion is a smooth, continuous curve. */
  private updateSimPoint(): void {
    const next: Map<string, Vec3> = new Map();
    for (const p of this.plans) {
      if (!p.visible) continue;
      const tr = p.trace;
      if (!tr?.t?.length) continue;
      const idx = lowerBound(tr.t, this.simTime);
      let r: Vec3;
      if (idx <= 0) {
        r = tr.pos[0];
      } else if (idx >= tr.t.length) {
        r = tr.pos[tr.pos.length - 1];
      } else {
        const t1 = tr.t[idx - 1];
        const t2 = tr.t[idx];
        const r1 = tr.pos[idx - 1];
        const r2 = tr.pos[idx];
        const f = (this.simTime - t1) / Math.max(1e-9, t2 - t1);
        // Position is read from the C⁵ trace; we only blend the two
        // adjacent samples linearly to give a sub-sample resolution.
        r = [
          r1[0] + (r2[0] - r1[0]) * f,
          r1[1] + (r2[1] - r1[1]) * f,
          r1[2] + (r2[2] - r1[2]) * f,
        ];
      }
      next.set(p.id, r);
    }
    this.simPoints = next;
  }

  private pushSimTime(t: number | null): void {
    this.fp.setSimTime(t).subscribe(() => { /* server echoed back */ });
  }
}

function lowerBound(arr: number[], target: number): number {
  let lo = 0, hi = arr.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < target) lo = mid + 1; else hi = mid;
  }
  return lo;
}
