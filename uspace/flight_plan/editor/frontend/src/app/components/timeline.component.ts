import {
  ChangeDetectionStrategy,
  Component,
  EventEmitter,
  Input,
  OnChanges,
  Output,
  SimpleChanges,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

import { FlightPlan } from '../services/flight-plan.service';

/** Bottom timeline strip.
 *  Shows every waypoint of every visible flight plan as a labelled
 *  tick, plus a draggable playhead for `simTime`. Resets the
 *  simulation back to 0 with the dedicated button. */
@Component({
  selector: 'app-timeline',
  standalone: true,
  imports: [CommonModule, FormsModule],
  changeDetection: ChangeDetectionStrategy.OnPush,
  template: `
    <div class="bar">
      <div class="left">
        <button class="btn danger" (click)="reset.emit()" title="Reset simulation to t=0">
          ⟲ Reset
        </button>
        <button class="btn primary" (click)="togglePlay()">
          {{ running ? '⏸' : '▶' }} {{ running ? 'Pause' : 'Play' }}
        </button>
        <label class="speed">
          speed
          <input class="input"
                 type="number"
                 step="0.1"
                 min="0"
                 [ngModel]="speed"
                 (change)="onSpeed($event)" />
          ×
        </label>
        <span class="time">t = <b>{{ simTime.toFixed(2) }}s</b></span>
        <span class="t-max">/ {{ tMax.toFixed(1) }}s</span>
      </div>

      <div class="track"
           #track
           (pointerdown)="onPointerDown($event, track)"
           (pointermove)="onPointerMove($event, track)"
           (pointerup)="onPointerUp()"
           (pointercancel)="onPointerUp()">
        <div class="rail"></div>
        <div class="fill" [style.width.%]="(simTime / (tMax || 1)) * 100"></div>
        <div class="wp-tick"
             *ngFor="let t of wpTicks"
             [style.left.%]="(t.time / (tMax || 1)) * 100"
             [style.background]="t.color"
             [title]="t.id + ' @ ' + t.time.toFixed(2) + 's'">
        </div>
        <div class="playhead" [style.left.%]="(simTime / (tMax || 1)) * 100"></div>
        <div class="ticks-row">
          <span *ngFor="let g of gridTicks" class="tick-label" [style.left.%]="(g / (tMax || 1)) * 100">
            {{ g.toFixed(1) }}s
          </span>
        </div>
      </div>
    </div>
  `,
  styles: [`
    :host { display: block; }
    .bar {
      display: grid;
      grid-template-columns: auto 1fr;
      gap: 16px;
      padding: 8px 12px;
      background: var(--bg-1);
      border-top: 1px solid var(--border-soft);
      align-items: center;
      height: 56px;
    }
    .left { display: flex; gap: 8px; align-items: center; }
    .speed { font-size: 11px; color: var(--fg-2); display: inline-flex; align-items: center; gap: 4px; }
    .speed input { width: 60px; }
    .time { font-family: 'JetBrains Mono', ui-monospace, monospace; font-size: 12px; color: var(--fg-0); margin-left: 6px; }
    .time b { color: var(--accent-2); }
    .t-max { color: var(--fg-3); font-size: 11px; margin-left: 2px; }
    .track {
      position: relative;
      height: 28px;
      cursor: pointer;
      user-select: none;
      touch-action: none;
    }
    .rail {
      position: absolute;
      left: 0; right: 0; top: 13px;
      height: 2px;
      background: var(--bg-3);
      border-radius: 2px;
    }
    .fill {
      position: absolute;
      left: 0; top: 13px;
      height: 2px;
      background: linear-gradient(90deg, var(--accent), var(--teal));
      border-radius: 2px;
    }
    .wp-tick {
      position: absolute;
      width: 2px;
      height: 18px;
      top: 5px;
      transform: translateX(-1px);
      border-radius: 2px;
    }
    .playhead {
      position: absolute;
      width: 14px; height: 14px;
      left: 0; top: 7px;
      transform: translateX(-7px);
      border-radius: 50%;
      background: var(--accent);
      box-shadow: 0 0 0 4px var(--accent-soft);
      transition: left 0.06s linear;
    }
    .ticks-row {
      position: absolute;
      bottom: -2px;
      left: 0; right: 0;
      height: 16px;
      pointer-events: none;
    }
    .tick-label {
      position: absolute;
      font-size: 9.5px;
      color: var(--fg-3);
      transform: translateX(-50%);
      font-family: 'JetBrains Mono', ui-monospace, monospace;
    }
  `],
})
export class TimelineComponent implements OnChanges {
  @Input() plans: FlightPlan[] = [];
  @Input() simTime = 0;
  @Input() tMax = 0;
  @Input() running = false;
  @Input() speed = 1;

  @Output() simTimeChange = new EventEmitter<number>();
  @Output() reset         = new EventEmitter<void>();
  @Output() playToggle    = new EventEmitter<void>();
  @Output() speedChange   = new EventEmitter<number>();

  wpTicks: { time: number; id: string; color: string }[] = [];
  gridTicks: number[] = [];

  private scrubbing = false;

  ngOnChanges(changes: SimpleChanges): void {
    const all: { time: number; id: string; color: string }[] = [];
    for (const p of this.plans) {
      if (!p.visible) continue;
      const color = colorFor(this.plans.indexOf(p));
      for (const w of p.waypoints) {
        all.push({ time: w.time, id: w.id, color });
      }
    }
    this.wpTicks = all;
    // grid ticks every 2s (or every 5s when > 50s)
    const step = this.tMax > 50 ? 5 : 2;
    const arr: number[] = [];
    for (let t = 0; t <= this.tMax + 0.001; t += step) arr.push(+t.toFixed(2));
    this.gridTicks = arr;
  }

  togglePlay(): void {
    this.playToggle.emit();
  }

  onSpeed(ev: Event): void {
    const v = parseFloat((ev.target as HTMLInputElement).value);
    if (Number.isFinite(v)) this.speedChange.emit(v);
  }

  onPointerDown(ev: PointerEvent, track: HTMLElement): void {
    (track as HTMLElement).setPointerCapture(ev.pointerId);
    this.scrubbing = true;
    this.updateFromPointer(ev, track);
  }
  onPointerMove(ev: PointerEvent, track: HTMLElement): void {
    if (!this.scrubbing) return;
    this.updateFromPointer(ev, track);
  }
  onPointerUp(): void {
    this.scrubbing = false;
  }

  private updateFromPointer(ev: PointerEvent, track: HTMLElement): void {
    if (!this.tMax) return;
    const rect = track.getBoundingClientRect();
    const x = Math.max(0, Math.min(rect.width, ev.clientX - rect.left));
    const t = (x / rect.width) * this.tMax;
    this.simTimeChange.emit(+t.toFixed(3));
  }
}

function colorFor(i: number): string {
  const PALETTE = ['#7c5cff', '#2dd4bf', '#fbbf24', '#f43f5e', '#5eead4', '#a78bfa'];
  return PALETTE[i % PALETTE.length];
}
