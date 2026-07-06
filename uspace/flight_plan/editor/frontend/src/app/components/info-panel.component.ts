import { ChangeDetectionStrategy, Component, EventEmitter, Input, Output } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

import { FlightPlan, Vec3, Waypoint } from '../services/flight-plan.service';

/** Technical-data panel: plan attributes + per-waypoint table. */
@Component({
  selector: 'app-info-panel',
  standalone: true,
  imports: [CommonModule, FormsModule],
  changeDetection: ChangeDetectionStrategy.OnPush,
  template: `
    <div class="panel-root">
      <!-- Plan header -->
      <header class="plan-header">
        <div class="row">
          <span class="dot" [style.background]="color"></span>
          <input class="input id-input"
                 [ngModel]="plan?.id ?? ''"
                 [readonly]="true"
                 title="Plan id" />
        </div>
        <div class="row-attr">
          <label>priority
            <input class="input"
                   type="number"
                   [ngModel]="plan?.priority ?? 0"
                   (change)="onPatchAttr($event, 'priority')" />
          </label>
          <label>radius
            <input class="input"
                   type="number"
                   step="0.1"
                   [ngModel]="plan?.radius ?? 1.0"
                   (change)="onPatchAttr($event, 'radius')" />
          </label>
          <label>lin vel
            <input class="input"
                   type="number"
                   step="0.1"
                   [ngModel]="plan?.max_var_lin_vel ?? 5"
                   (change)="onPatchAttr($event, 'max_var_lin_vel')" />
          </label>
          <label>ang vel
            <input class="input"
                   type="number"
                   step="0.1"
                   [ngModel]="plan?.max_var_ang_vel ?? 1"
                   (change)="onPatchAttr($event, 'max_var_ang_vel')" />
          </label>
        </div>
        <div class="row-meta">
          <span class="chip">{{ plan?.length ?? 0 }} WP</span>
          <span class="chip">start: {{ fmtTime(plan?.start_time) }}</span>
          <span class="chip">finish: {{ fmtTime(plan?.finish_time) }}</span>
          <span class="spacer"></span>
          <button class="btn" (click)="addWaypoint.emit()">+ WP</button>
          <button class="btn primary" (click)="connect.emit()" title="Re-derive jerk/snap/crackle">Connect</button>
        </div>
      </header>

      <!-- Waypoint table -->
      <div class="table-wrap">
        <table class="wp-table">
          <thead>
            <tr>
              <th>id</th>
              <th>time (s)</th>
              <th>x (m)</th>
              <th>y (m)</th>
              <th>z (m)</th>
              <th>vx</th>
              <th>vy</th>
              <th>vz</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            <tr *ngFor="let w of plan?.waypoints ?? []; let i = index"
                [class.selected]="selectedId === w.id"
                (click)="select.emit(w.id)">
              <td>
                <input class="input id-cell"
                       [ngModel]="w.id"
                       (change)="onPatchId(w.id, $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.01"
                       [ngModel]="w.time"
                       (change)="onPatchField(w.id, 'time', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.pos[0]"
                       (change)="onPatchVec(w.id, 0, 'pos', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.pos[1]"
                       (change)="onPatchVec(w.id, 1, 'pos', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.pos[2]"
                       (change)="onPatchVec(w.id, 2, 'pos', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.vel[0]"
                       (change)="onPatchVec(w.id, 0, 'vel', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.vel[1]"
                       (change)="onPatchVec(w.id, 1, 'vel', $event)" />
              </td>
              <td>
                <input class="input num"
                       type="number" step="0.1"
                       [ngModel]="w.vel[2]"
                       (change)="onPatchVec(w.id, 2, 'vel', $event)" />
              </td>
              <td>
                <button class="btn danger small"
                        (click)="onDeleteWp(w.id, $event)"
                        title="Delete waypoint">×</button>
              </td>
            </tr>
            <tr *ngIf="(plan?.waypoints?.length ?? 0) === 0">
              <td colspan="9" class="empty">
                No waypoints. Click on the 3D viewer (tool "Add WP") or press the <span class="kbd">+ WP</span> button above.
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  `,
  styles: [`
    :host { display: block; width: 100%; height: 100%; min-height: 0; }
    .panel-root {
      display: flex;
      flex-direction: column;
      height: 100%;
      min-height: 0;
      background: var(--bg-1);
      border-radius: var(--radius);
      border: 1px solid var(--border-soft);
      overflow: hidden;
    }
    .plan-header {
      padding: 10px 12px;
      background: var(--bg-1);
      border-bottom: 1px solid var(--border-soft);
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .row { display: flex; align-items: center; gap: 8px; }
    .row-attr { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; }
    .row-attr label { display: flex; flex-direction: column; font-size: 10px; color: var(--fg-2); }
    .row-meta { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
    .id-input { max-width: 200px; font-weight: 600; color: var(--accent-2); }
    .row-meta .spacer { flex: 1; }
    .table-wrap { overflow: auto; flex: 1; min-height: 0; }
    .wp-table {
      width: 100%;
      border-collapse: collapse;
      font-family: 'JetBrains Mono', ui-monospace, monospace;
      font-size: 11.5px;
    }
    .wp-table thead th {
      position: sticky;
      top: 0;
      background: var(--bg-2);
      color: var(--fg-2);
      font-weight: 600;
      text-align: left;
      padding: 6px 6px;
      border-bottom: 1px solid var(--border);
      font-size: 10.5px;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }
    .wp-table tbody td { padding: 2px 4px; border-bottom: 1px solid var(--border-soft); }
    .wp-table tbody tr { transition: background var(--t-fast); }
    .wp-table tbody tr:hover { background: rgba(124, 92, 255, 0.06); }
    .wp-table tbody tr.selected { background: var(--accent-soft); }
    .wp-table .input { padding: 3px 6px; }
    .wp-table .id-cell { color: var(--accent-2); font-weight: 600; min-width: 80px; }
    .wp-table .num { text-align: right; min-width: 60px; }
    .btn.small { padding: 2px 8px; font-size: 12px; }
    .empty { color: var(--fg-2); text-align: center; padding: 24px; }
  `],
})
export class InfoPanelComponent {
  @Input() plan: FlightPlan | null = null;
  @Input() selectedId: string | null = null;
  /** Color used to identify the current plan in the table's id dot. */
  @Input() color = '#7c5cff';

  @Output() select       = new EventEmitter<string>();
  @Output() addWaypoint  = new EventEmitter<void>();
  @Output() connect      = new EventEmitter<void>();
  @Output() deleteWp     = new EventEmitter<string>();
  @Output() updateWp     = new EventEmitter<Waypoint>();
  @Output() updateAttr   = new EventEmitter<{ attr: string; value: any }>();

  onPatchField(wpId: string, field: 'time', ev: Event): void {
    const value = parseFloat((ev.target as HTMLInputElement).value);
    const wp = this.plan?.waypoints.find((w) => w.id === wpId);
    if (!wp || !Number.isFinite(value)) return;
    this.updateWp.emit({ ...wp, time: value });
  }

  onPatchVec(wpId: string, axis: 0 | 1 | 2, kind: 'pos' | 'vel', ev: Event): void {
    const value = parseFloat((ev.target as HTMLInputElement).value);
    const wp = this.plan?.waypoints.find((w) => w.id === wpId);
    if (!wp || !Number.isFinite(value)) return;
    const arr: Vec3 = kind === 'pos' ? [...wp.pos] : [...wp.vel];
    arr[axis] = value;
    this.updateWp.emit({ ...wp, [kind]: arr } as Waypoint);
  }

  onPatchAttr(ev: Event, attr: string): void {
    const value = parseFloat((ev.target as HTMLInputElement).value);
    if (!Number.isFinite(value)) return;
    this.updateAttr.emit({ attr, value });
  }

  onPatchId(oldId: string, ev: Event): void {
    // id is read-only — the per-row input is for display only. Snap
    // back any edit the user managed to make (the input is `readonly`
    // in the template, this is a belt-and-braces guard).
    (ev.target as HTMLInputElement).value = oldId;
  }

  onDeleteWp(id: string, ev: Event): void {
    ev.stopPropagation();
    this.deleteWp.emit(id);
  }

  fmtTime(t: number | null | undefined): string {
    if (t == null) return '—';
    return `${(t as number).toFixed(2)}s`;
  }
}
