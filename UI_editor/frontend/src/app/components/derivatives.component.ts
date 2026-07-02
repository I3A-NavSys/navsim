import {
  AfterViewInit,
  Component,
  ElementRef,
  Input,
  OnChanges,
  SimpleChanges,
  ViewChild,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Trace, Waypoint } from '../services/flight-plan.service';

interface PlotSpec {
  key: 'vel' | 'acel' | 'jerk' | 'snap' | 'crakle';
  label: string;
  unit:  string;
  color: string;
}

const SPECS: PlotSpec[] = [
  { key: 'vel',    label: 'Velocity',     unit: 'm/s',   color: '#4caf50' },
  { key: 'acel',   label: 'Acceleration', unit: 'm/s²',  color: '#ff9800' },
  { key: 'jerk',   label: 'Jerk',         unit: 'm/s³',  color: '#f44336' },
  { key: 'snap',   label: 'Snap',         unit: 'm/s⁴',  color: '#9c27b0' },
  { key: 'crakle', label: 'Crackle',      unit: 'm/s⁵',  color: '#03a9f4' },
];

/**
 * Draws the trace's derivatives over time, with a per-derivative
 * visibility checkbox and a per-axis toggle (x, y, z, |·|).
 *
 * Vertical lines are drawn at every waypoint's time so the user can
 * see which region of each derivative curve corresponds to which
 * waypoint.
 */
@Component({
  selector: 'app-derivatives',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="der-panel">
      <div class="der-header" (click)="collapsed = !collapsed">
        <span>Derivatives</span>
        <button class="collapse-btn">{{ collapsed ? '▴' : '▾' }}</button>
      </div>

      <div *ngIf="!collapsed" class="der-body">
        <div class="der-controls">
          <span class="ctrl-label">Show:</span>
          <label *ngFor="let s of specs" class="der-check" [style.color]="s.color">
            <input type="checkbox"
                   [(ngModel)]="visible[s.key]"
                   (ngModelChange)="redraw()">
            {{ s.label }} <small>({{ s.unit }})</small>
          </label>
        </div>
        <div class="der-controls">
          <span class="ctrl-label">Axes:</span>
          <label class="der-check axis-x">
            <input type="checkbox" [(ngModel)]="showX" (ngModelChange)="redraw()">x
          </label>
          <label class="der-check axis-y">
            <input type="checkbox" [(ngModel)]="showY" (ngModelChange)="redraw()">y
          </label>
          <label class="der-check axis-z">
            <input type="checkbox" [(ngModel)]="showZ" (ngModelChange)="redraw()">z
          </label>
          <label class="der-check axis-mag">
            <input type="checkbox" [(ngModel)]="showMag" (ngModelChange)="redraw()">|·|
          </label>
        </div>

        <div class="der-canvas-wrap">
          <canvas #cvs class="der-canvas"></canvas>
        </div>

        <div class="der-legend" *ngIf="waypoints && waypoints.length > 0">
          <span class="ctrl-label">Waypoints:</span>
          <span *ngFor="let wp of waypoints" class="wp-chip">
            <span class="wp-tick" [style.background]="colorFor(wp)"></span>
            {{ wp.id }} &#64; t={{ wp.time | number:'1.1-2' }}s
          </span>
        </div>
      </div>
    </div>
  `,
  styles: [`
    .der-panel {
      background: #1f1f24;
      border: 1px solid #444;
      border-radius: 4px;
      margin-top: 8px;
      color: #ddd;
      font-size: 12px;
    }
    .der-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 6px 10px;
      cursor: pointer;
      user-select: none;
      font-weight: 500;
    }
    .der-header:hover { background: #2a2a30; }
    .collapse-btn {
      background: none;
      border: none;
      color: #aaa;
      cursor: pointer;
      font-size: 14px;
    }
    .der-body {
      padding: 6px 10px 10px 10px;
      border-top: 1px solid #333;
    }
    .der-controls {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 10px;
      margin-bottom: 4px;
    }
    .ctrl-label { color: #888; font-size: 11px; min-width: 50px; }
    .der-check {
      display: flex;
      align-items: center;
      gap: 4px;
      font-size: 12px;
      color: #ccc;
    }
    .der-check.axis-x { color: #ff6e6e; }
    .der-check.axis-y { color: #66ff8c; }
    .der-check.axis-z { color: #6e9eff; }
    .der-check.axis-mag { color: #ffcc33; }
    .der-canvas-wrap {
      width: 100%;
      height: 220px;
      background: #15151a;
      border: 1px solid #333;
      border-radius: 3px;
      margin-top: 6px;
    }
    .der-canvas { width: 100%; height: 100%; display: block; }
    .der-legend {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 10px;
      margin-top: 6px;
      font-size: 11px;
      color: #aaa;
    }
    .wp-chip {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      padding: 1px 6px;
      background: #2a2a30;
      border-radius: 3px;
      color: #ddd;
    }
    .wp-tick {
      display: inline-block;
      width: 6px;
      height: 12px;
      background: #ffcc33;
      border-radius: 1px;
    }
  `],
})
export class DerivativesComponent implements AfterViewInit, OnChanges {
  @ViewChild('cvs', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  @Input() trace: Trace | null = null;
  @Input() waypoints: Waypoint[] = [];

  specs = SPECS;
  collapsed = false;

  visible: Record<string, boolean> = {
    vel: true, acel: true, jerk: true, snap: false, crakle: false,
  };
  showX = true;
  showY = true;
  showZ = false;
  showMag = true;

  private ctx!: CanvasRenderingContext2D;
  private dpr = 1;
  private cssW = 0;
  private cssH = 0;

  ngAfterViewInit(): void {
    const cvs = this.canvasRef.nativeElement;
    this.ctx = cvs.getContext('2d')!;
    this.dpr = window.devicePixelRatio || 1;
    this.resize();
    window.addEventListener('resize', () => { this.resize(); this.redraw(); });
    this.redraw();
  }

  ngOnChanges(_: SimpleChanges): void { this.redraw(); }

  /** Stable color per WP — derived from its index in the list. */
  colorFor(wp: Waypoint): string {
    const i = this.waypoints.findIndex((w) => w.id === wp.id);
    if (i < 0) return '#ffcc33';
    const hues = [48, 30, 200, 280, 130, 0, 60, 320, 180, 240];
    return `hsl(${hues[i % hues.length]}, 80%, 60%)`;
  }

  private resize(): void {
    const cvs = this.canvasRef.nativeElement;
    const rect = cvs.getBoundingClientRect();
    this.cssW = rect.width;
    this.cssH = rect.height;
    cvs.width  = Math.max(1, Math.floor(this.cssW * this.dpr));
    cvs.height = Math.max(1, Math.floor(this.cssH * this.dpr));
  }

  redraw(): void {
    if (!this.ctx) return;
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.cssW, this.cssH);

    if (!this.trace || this.trace.t.length === 0) {
      this.drawCenteredText('No trace yet. Press "Refresh trace" after editing waypoints.');
      return;
    }

    const t   = this.trace.t;
    const w   = this.cssW;
    const h   = this.cssH;
    const pad = 28;
    const tMin = t[0];
    const tMax = t[t.length - 1];

    let yMax = 0.0001;
    for (const s of this.specs) {
      if (!this.visible[s.key]) continue;
      const series = (this.trace as any)[s.key] as number[][];
      for (let i = 0; i < series.length; i++) {
        const v = series[i];
        if (this.showX) yMax = Math.max(yMax, Math.abs(v[0]));
        if (this.showY) yMax = Math.max(yMax, Math.abs(v[1]));
        if (this.showZ) yMax = Math.max(yMax, Math.abs(v[2]));
        if (this.showMag) yMax = Math.max(yMax, Math.hypot(v[0], v[1], v[2]));
      }
    }

    const xToPx = (ti: number) => pad + (ti - tMin) / (tMax - tMin) * (w - pad - 6);
    const yToPx = (vi: number) => (h - pad) - (vi / yMax) * (h - 2 * pad);

    // ---- 1. Axes ------------------------------------------------------
    ctx.strokeStyle = '#444';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.moveTo(pad, h - pad);
    ctx.lineTo(w - pad / 2, h - pad);
    ctx.moveTo(pad, pad / 2);
    ctx.lineTo(pad, h - pad);
    ctx.stroke();

    // Axis labels.
    ctx.fillStyle = '#888';
    ctx.font      = '10px monospace';
    ctx.fillText(`t [s] (${tMin.toFixed(1)} → ${tMax.toFixed(1)})`, w - 220, h - 6);
    ctx.fillText(`yMax=${yMax.toFixed(2)}`, 4, 12);

    // ---- 2. Waypoint time markers (drawn behind the curves) ---------
    if (this.waypoints && this.waypoints.length > 0) {
      for (let i = 0; i < this.waypoints.length; i++) {
        const wp = this.waypoints[i];
        if (wp.time < tMin || wp.time > tMax) continue;
        const x = xToPx(wp.time);
        ctx.strokeStyle = this.colorFor(wp);
        ctx.globalAlpha = 0.35;
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(x, pad / 2);
        ctx.lineTo(x, h - pad);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1.0;
        // ID label at the top.
        ctx.fillStyle = this.colorFor(wp);
        ctx.font = '10px monospace';
        ctx.fillText(wp.id, x + 3, pad / 2 + 10);
      }
    }

    // ---- 3. Derivatives ---------------------------------------------
    for (const s of this.specs) {
      if (!this.visible[s.key]) continue;
      const series = (this.trace as any)[s.key] as number[][];
      this.drawSeries(ctx, t, series, s.color, xToPx, yToPx);
    }
  }

  private drawSeries(
    ctx: CanvasRenderingContext2D,
    times: number[],
    series: number[][],
    color: string,
    xToPx: (t: number) => number,
    yToPx: (v: number) => number,
  ): void {
    if (this.showMag) {
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.25;
      ctx.lineWidth   = 1.4;
      ctx.beginPath();
      for (let i = 0; i < series.length; i++) {
        const v = series[i];
        const m = Math.hypot(v[0], v[1], v[2]);
        const px = xToPx(times[i]);
        const py = yToPx(m);
        if (i === 0) ctx.moveTo(px, py);
        else         ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    }

    const axes: { on: boolean; idx: number; color: string }[] = [
      { on: this.showX, idx: 0, color: 'rgba(255,110,110,0.9)' },
      { on: this.showY, idx: 1, color: 'rgba(102,255,140,0.9)' },
      { on: this.showZ, idx: 2, color: 'rgba(110,158,255,0.9)' },
    ];

    for (const a of axes) {
      if (!a.on) continue;
      ctx.strokeStyle = a.color;
      ctx.lineWidth   = 1.6;
      ctx.beginPath();
      for (let i = 0; i < series.length; i++) {
        const v = series[i][a.idx];
        const px = xToPx(times[i]);
        const py = yToPx(v);
        if (i === 0) ctx.moveTo(px, py);
        else         ctx.lineTo(px, py);
      }
      ctx.stroke();
    }
  }

  private drawCenteredText(text: string): void {
    const ctx = this.ctx;
    ctx.fillStyle = '#666';
    ctx.font      = '12px monospace';
    const w = this.cssW;
    const tw = ctx.measureText(text).width;
    ctx.fillText(text, (w - tw) / 2, this.cssH / 2);
  }
}
