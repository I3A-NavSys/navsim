import { Component, EventEmitter, Input, Output } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Waypoint } from '../services/flight-plan.service';

interface EditableWp {
  id: string;
  time: number;
  pos: [number, number, number];
  vel: [number, number, number];
  acel: [number, number, number];
}

@Component({
  selector: 'app-info-panel',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="panel">
      <h3>Flight Plan</h3>
      <div class="kv">
        <span>ID:</span><b>{{ planId }}</b>
        <span>Length:</span><b>{{ waypoints.length }} waypoints</b>
        <span>Start time:</span><b>{{ startTime | number:'1.1-2' }} s</b>
        <span>Finish time:</span><b>{{ finishTime | number:'1.1-2' }} s</b>
        <span>Total length:</span><b>{{ totalLength | number:'1.1-2' }} m</b>
      </div>

      <div class="actions">
        <button (click)="reset.emit()">Reset</button>
        <button (click)="connect.emit()">Connect</button>
        <button (click)="refreshTrace.emit()">Refresh trace</button>
        <label class="z-slider">
          Default z:
          <input type="range" min="0" max="200" step="1"
                 [ngModel]="defaultZ"
                 (ngModelChange)="defaultZChange.emit($event)">
          <span>{{ defaultZ }} m</span>
        </label>
      </div>

      <h3>Waypoints</h3>
      <p class="hint">
        Each row edits one waypoint. Clicking "Update" sends a single
        PATCH to the backend so the in-memory FlightPlan and the trace
        are refreshed atomically.
      </p>
      <table>
        <thead>
          <tr>
            <th>ID</th>
            <th>t [s]</th>
            <th>x [m]</th>
            <th>y [m]</th>
            <th>z [m]</th>
            <th>vx</th>
            <th>vy</th>
            <th>vz</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          <tr *ngFor="let w of editable; trackBy: trackById"
              [class.selected]="w.id === selectedId"
              (click)="select.emit(w.id)">
            <td>{{ w.id }}</td>
            <td><input type="number" step="0.5"  [(ngModel)]="w.time" (change)="sync(w)"></td>
            <td><input type="number" step="0.5"  [(ngModel)]="w.pos[0]" (change)="sync(w)"></td>
            <td><input type="number" step="0.5"  [(ngModel)]="w.pos[1]" (change)="sync(w)"></td>
            <td><input type="number" step="0.5"  [(ngModel)]="w.pos[2]" (change)="sync(w)"></td>
            <td><input type="number" step="0.1"  [(ngModel)]="w.vel[0]" (change)="sync(w)"></td>
            <td><input type="number" step="0.1"  [(ngModel)]="w.vel[1]" (change)="sync(w)"></td>
            <td><input type="number" step="0.1"  [(ngModel)]="w.vel[2]" (change)="sync(w)"></td>
            <td><button (click)="deleteWp.emit(w.id); $event.stopPropagation()">×</button></td>
          </tr>
        </tbody>
      </table>
    </div>
  `,
  styles: [`
    :host { display: block; }
    .panel {
      background: #1f1f24;
      border: 1px solid #444;
      border-radius: 4px;
      padding: 10px;
      color: #ddd;
      font-size: 12px;
    }
    h3 { margin: 0 0 8px 0; font-size: 14px; color: #ffcc33; }
    .kv {
      display: grid;
      grid-template-columns: 110px 1fr;
      gap: 4px 12px;
      margin-bottom: 12px;
    }
    .kv span { color: #aaa; }
    .kv b    { color: #fff; font-weight: 500; }
    .actions {
      display: flex;
      gap: 8px;
      align-items: center;
      flex-wrap: wrap;
      margin-bottom: 12px;
      padding-bottom: 10px;
      border-bottom: 1px solid #333;
    }
    .actions button {
      background: #3a3a44;
      color: #ddd;
      border: 1px solid #555;
      padding: 4px 10px;
      border-radius: 3px;
      cursor: pointer;
      font-size: 12px;
    }
    .actions button:hover { background: #4a4a55; }
    .z-slider {
      display: flex;
      align-items: center;
      gap: 6px;
      color: #ccc;
      font-size: 12px;
    }
    .z-slider input { width: 130px; }
    .z-slider span  { color: #fff; min-width: 50px; }
    .hint {
      color: #888;
      font-size: 11px;
      margin: 0 0 6px 0;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 11px;
    }
    th, td {
      padding: 2px 4px;
      text-align: left;
      border-bottom: 1px solid #2a2a30;
    }
    th { color: #888; font-weight: 500; }
    tr.selected { background: #2a72c4; }
    tr.selected td input { color: #fff; }
    input[type=number] {
      background: #15151a;
      color: #fff;
      border: 1px solid #444;
      border-radius: 3px;
      padding: 2px 4px;
      width: 60px;
      font-family: monospace;
    }
    button {
      background: #b04c4c;
      color: #fff;
      border: none;
      padding: 2px 8px;
      border-radius: 3px;
      cursor: pointer;
    }
    button:hover { background: #c95a5a; }
  `],
})
export class InfoPanelComponent {
  @Input() planId = '';
  @Input() waypoints: Waypoint[] = [];
  @Input() startTime = 0;
  @Input() finishTime = 0;
  @Input() totalLength = 0;
  @Input() defaultZ = 50;
  @Input() selectedId: string | null = null;

  @Output() reset          = new EventEmitter<void>();
  @Output() connect        = new EventEmitter<void>();
  @Output() refreshTrace   = new EventEmitter<void>();
  @Output() defaultZChange = new EventEmitter<number>();
  @Output() updateWp       = new EventEmitter<Waypoint>();
  @Output() deleteWp       = new EventEmitter<string>();
  @Output() select         = new EventEmitter<string>();

  /** Local editable copy of `waypoints` so the table can mutate cells
   *  and we emit the full update when the user moves focus away. */
  editable: EditableWp[] = [];

  ngOnChanges(): void {
    // Rebuild the editable list whenever the input changes.
    this.editable = this.waypoints.map((w) => ({
      id:    w.id,
      time:  w.time,
      pos:   [...w.pos]   as [number, number, number],
      vel:   [...w.vel]   as [number, number, number],
      acel:  [...w.acel]  as [number, number, number],
    }));
  }

  sync(w: EditableWp): void {
    this.updateWp.emit({
      id:    w.id,
      time:  w.time,
      pos:   w.pos,
      vel:   w.vel,
      acel:  w.acel,
      jerk:  [0, 0, 0],
      snap:  [0, 0, 0],
      crakle:[0, 0, 0],
    });
  }

  trackById(_: number, w: EditableWp): string { return w.id; }
}
