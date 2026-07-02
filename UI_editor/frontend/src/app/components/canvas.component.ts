import {
  AfterViewInit,
  Component,
  ElementRef,
  EventEmitter,
  Input,
  OnChanges,
  Output,
  SimpleChanges,
  ViewChild,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { Waypoint } from '../services/flight-plan.service';

type Tool = 'add' | 'move' | 'vel';

interface DragState {
  kind: 'wp-move' | 'vel' | 'pan';
  wpId?: string;
  startMouse: { x: number; y: number };
  startValue?: any;
  startOrigin?: [number, number];
}

const WP_RADIUS = 8;
const VEL_HEAD_LEN = 14;
const VEL_HEAD_WID = 7;
const MIN_PPM = 1;
const MAX_PPM = 60;

/**
 * Canvas-based 2D editor for the FlightPlan.
 *
 * Two view transforms are exposed to the outside world so the parent
 * can synchronise them (e.g. when the user clicks "Fit"):
 *   - `pixelsPerMeter` : how many screen pixels one metre takes.
 *   - `worldOrigin`    : the (x, y) world point rendered at the
 *                         bottom-left of the canvas.
 *
 * The user can:
 *   1) click on the canvas to add a waypoint (tool == "add")
 *   2) drag a waypoint to move it          (tool == "move")
 *   3) drag the tip of the velocity vector to scale / rotate it
 *                                       (tool == "vel")
 *   4) scroll the wheel to zoom in / out around the cursor
 *   5) middle-click drag (or shift+drag) to pan the view
 */
@Component({
  selector: 'app-canvas',
  standalone: true,
  imports: [CommonModule],
  template: `
    <div class="canvas-toolbar">
      <span class="tool-label">Tool:</span>
      <button [class.active]="tool === 'add'"  (click)="setTool('add')">Add WP</button>
      <button [class.active]="tool === 'move'" (click)="setTool('move')">Move WP</button>
      <button [class.active]="tool === 'vel'"  (click)="setTool('vel')">Edit Velocity</button>
      <span class="spacer"></span>
      <label class="check">
        <input type="checkbox" [checked]="showVelocity"
               (change)="showVelocityChange.emit($any($event.target).checked)">
        Velocity
      </label>
      <label class="check">
        <input type="checkbox" [checked]="showTrace"
               (change)="showTraceChange.emit($any($event.target).checked)">
        Trace
      </label>
    </div>

    <canvas #cvs
            class="fp-canvas"
            (mousedown)="onMouseDown($event)"
            (mousemove)="onMouseMove($event)"
            (mouseup)="onMouseUp($event)"
            (mouseleave)="onMouseUp($event)"
            (wheel)="onWheel($event)"
            (contextmenu)="$event.preventDefault()">
    </canvas>

    <div class="canvas-info">
      <span>Cursor: ({{ cursorX | number:'1.1-2' }}, {{ cursorY | number:'1.1-2' }}) m</span>
      <span>Origin: ({{ worldOrigin[0] | number:'1.0-1' }}, {{ worldOrigin[1] | number:'1.0-1' }}) m</span>
      <span>Zoom: {{ pixelsPerMeter | number:'1.1-2' }} px/m</span>
      <span class="hint">Wheel = zoom · Shift+drag = pan</span>
    </div>
  `,
  styles: [`
    :host {
      display: flex;
      flex-direction: column;
      gap: 6px;
      height: 100%;
      min-height: 320px;
    }
    .canvas-toolbar {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 4px;
      background: #2a2a30;
      border-radius: 4px;
    }
    .tool-label { color: #aaa; font-size: 12px; }
    .spacer { flex: 1; }
    button {
      background: #3a3a44;
      color: #ddd;
      border: 1px solid #555;
      padding: 4px 10px;
      border-radius: 3px;
      cursor: pointer;
      font-size: 12px;
    }
    button:hover { background: #4a4a55; }
    button.active { background: #2a72c4; border-color: #2a72c4; color: #fff; }
    .check {
      display: flex;
      align-items: center;
      gap: 4px;
      color: #ccc;
      font-size: 12px;
    }
    .fp-canvas {
      flex: 1;
      width: 100%;
      background: #1a1a1f;
      border: 1px solid #444;
      border-radius: 4px;
      cursor: crosshair;
    }
    .fp-canvas:active { cursor: grabbing; }
    .canvas-info {
      display: flex;
      gap: 14px;
      font-size: 11px;
      color: #888;
      flex-wrap: wrap;
    }
    .hint { color: #5a8a5a; font-style: italic; }
  `],
})
export class CanvasComponent implements AfterViewInit, OnChanges {
  @ViewChild('cvs', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  @Input() waypoints: Waypoint[] = [];
  @Input() tracePoints: [number, number][] = [];
  @Input() tool: Tool = 'add';
  @Input() showVelocity = true;
  @Input() showTrace = true;
  @Input() pixelsPerMeter = 6;
  @Input() worldOrigin: [number, number] = [0, 0];

  @Output() toolChange           = new EventEmitter<Tool>();
  @Output() showVelocityChange   = new EventEmitter<boolean>();
  @Output() showTraceChange      = new EventEmitter<boolean>();
  @Output() addWpRequested       = new EventEmitter<{ x: number; y: number }>();
  @Output() wpMoved              = new EventEmitter<{ id: string; pos: [number, number, number] }>();
  @Output() velChanged           = new EventEmitter<{ id: string; vel: [number, number, number] }>();
  @Output() zoomChanged          = new EventEmitter<number>();
  @Output() panChanged           = new EventEmitter<[number, number]>();

  cursorX = 0;
  cursorY = 0;

  private drag: DragState | null = null;
  private ctx!: CanvasRenderingContext2D;
  private dpr = 1;
  private cssW = 0;
  private cssH = 0;

  // Local copies of the zoom/pan so the wheel handler can mutate them
  // without round-tripping through the parent (the parent is notified
  // via the Output emitters).
  private ppm = 6;
  private origin: [number, number] = [0, 0];

  ngAfterViewInit(): void {
    const cvs = this.canvasRef.nativeElement;
    this.ctx = cvs.getContext('2d')!;
    this.dpr = window.devicePixelRatio || 1;
    this.resize();
    this.ppm    = this.pixelsPerMeter;
    this.origin = [...this.worldOrigin];
    window.addEventListener('resize', this.resize.bind(this));
    this.draw();
  }

  ngOnChanges(changes: SimpleChanges): void {
    if (changes['pixelsPerMeter']) this.ppm = this.pixelsPerMeter;
    if (changes['worldOrigin'])    this.origin = [...this.worldOrigin];
    if (this.ctx) this.draw();
  }

  setTool(t: Tool): void { this.tool = t; this.toolChange.emit(t); }

  // ---------- view transforms ----------
  private toWorld(clientX: number, clientY: number): { x: number; y: number } {
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    const cx = clientX - rect.left;
    const cy = clientY - rect.top;
    return {
      x: this.origin[0] + cx / this.ppm,
      y: this.origin[1] + cy / this.ppm,
    };
  }

  private toCanvas(x: number, y: number): { cx: number; cy: number } {
    return {
      cx: (x - this.origin[0]) * this.ppm,
      cy: (y - this.origin[1]) * this.ppm,
    };
  }

  // ---------- hit testing ----------
  private hitWaypoint(cx: number, cy: number): Waypoint | null {
    for (let i = this.waypoints.length - 1; i >= 0; i--) {
      const wp = this.waypoints[i];
      const p = this.toCanvas(wp.pos[0], wp.pos[1]);
      const dx = p.cx - cx;
      const dy = p.cy - cy;
      if (dx * dx + dy * dy < (WP_RADIUS + 4) ** 2) return wp;
    }
    return null;
  }

  private hitVelHandle(cx: number, cy: number): Waypoint | null {
    for (let i = this.waypoints.length - 1; i >= 0; i--) {
      const wp = this.waypoints[i];
      const p = this.toCanvas(wp.pos[0], wp.pos[1]);
      const tip = {
        cx: p.cx + wp.vel[0] * this.ppm,
        cy: p.cy + wp.vel[1] * this.ppm,
      };
      const dx = tip.cx - cx;
      const dy = tip.cy - cy;
      if (dx * dx + dy * dy < (VEL_HEAD_LEN / 2 + 4) ** 2) return wp;
    }
    return null;
  }

  // ---------- mouse handlers ----------
  onMouseDown(ev: MouseEvent): void {
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    const cx = ev.clientX - rect.left;
    const cy = ev.clientY - rect.top;

    // Middle button OR shift+left = pan.
    if (ev.button === 1 || (ev.button === 0 && ev.shiftKey)) {
      this.drag = {
        kind: 'pan',
        startMouse: { x: ev.clientX, y: ev.clientY },
        startOrigin: [...this.origin],
      };
      ev.preventDefault();
      return;
    }

    if (this.tool === 'add') {
      const w = this.toWorld(ev.clientX, ev.clientY);
      this.addWpRequested.emit({ x: w.x, y: w.y });
      return;
    }

    if (this.tool === 'vel') {
      const wp = this.hitVelHandle(cx, cy) ?? this.hitWaypoint(cx, cy);
      if (wp) {
        this.drag = {
          kind: 'vel',
          wpId: wp.id,
          startMouse: { x: ev.clientX, y: ev.clientY },
          startValue: [...wp.vel] as [number, number, number],
        };
      }
      return;
    }

    if (this.tool === 'move') {
      const wp = this.hitWaypoint(cx, cy);
      if (wp) {
        this.drag = {
          kind: 'wp-move',
          wpId: wp.id,
          startMouse: { x: ev.clientX, y: ev.clientY },
          startValue: [...wp.pos] as [number, number, number],
        };
      }
      return;
    }
  }

  onMouseMove(ev: MouseEvent): void {
    const w = this.toWorld(ev.clientX, ev.clientY);
    this.cursorX = w.x;
    this.cursorY = w.y;

    if (!this.drag) return;

    if (this.drag.kind === 'pan' && this.drag.startOrigin) {
      const dxPx = ev.clientX - this.drag.startMouse.x;
      const dyPx = ev.clientY - this.drag.startMouse.y;
      const newOrigin: [number, number] = [
        this.drag.startOrigin[0] - dxPx / this.ppm,
        this.drag.startOrigin[1] - dyPx / this.ppm,
      ];
      this.origin = newOrigin;
      this.panChanged.emit(newOrigin);
      this.draw();
      return;
    }

    if (this.drag.kind === 'wp-move' && this.drag.wpId) {
      const dxPx = ev.clientX - this.drag.startMouse.x;
      const dyPx = ev.clientY - this.drag.startMouse.y;
      const dx = dxPx / this.ppm;
      const dy = dyPx / this.ppm;
      const start = this.drag.startValue as [number, number, number];
      this.wpMoved.emit({
        id: this.drag.wpId,
        pos: [start[0] + dx, start[1] + dy, start[2]],
      });
    } else if (this.drag.kind === 'vel' && this.drag.wpId) {
      const wp = this.waypoints.find((w) => w.id === this.drag!.wpId);
      if (!wp) return;
      const w2 = this.toWorld(ev.clientX, ev.clientY);
      const vx = w2.x - wp.pos[0];
      const vy = w2.y - wp.pos[1];
      const start = this.drag.startValue as [number, number, number];
      this.velChanged.emit({
        id: this.drag.wpId,
        vel: [vx, vy, start[2]],
      });
    }
  }

  onMouseUp(_: MouseEvent): void {
    this.drag = null;
  }

  onWheel(ev: WheelEvent): void {
    ev.preventDefault();
    const factor = ev.deltaY < 0 ? 1.15 : 1 / 1.15;
    const newPpm = Math.max(MIN_PPM, Math.min(MAX_PPM, this.ppm * factor));
    if (newPpm === this.ppm) return;

    // Zoom around the cursor: the world point under the cursor must
    // stay at the same canvas pixel.
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    const cx = ev.clientX - rect.left;
    const cy = ev.clientY - rect.top;
    const worldX = this.origin[0] + cx / this.ppm;
    const worldY = this.origin[1] + cy / this.ppm;

    this.ppm = newPpm;
    this.origin = [
      worldX - cx / this.ppm,
      worldY - cy / this.ppm,
    ];
    this.zoomChanged.emit(this.ppm);
    this.panChanged.emit([...this.origin]);
    this.draw();
  }

  // ---------- drawing ----------
  private resize(): void {
    const cvs = this.canvasRef.nativeElement;
    const rect = cvs.getBoundingClientRect();
    this.cssW = rect.width;
    this.cssH = rect.height;
    cvs.width  = Math.max(1, Math.floor(this.cssW * this.dpr));
    cvs.height = Math.max(1, Math.floor(this.cssH * this.dpr));
  }

  private draw(): void {
    if (!this.ctx) return;
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.cssW, this.cssH);

    this.drawGrid();
    this.drawTrace();
    for (const wp of this.waypoints) this.drawWaypoint(wp);
  }

  private drawGrid(): void {
    const ctx = this.ctx;
    const step = 10;
    const w = this.cssW;
    const h = this.cssH;

    const minX = this.origin[0];
    const minY = this.origin[1];
    const maxX = minX + w / this.ppm;
    const maxY = minY + h / this.ppm;

    ctx.strokeStyle = '#2c2c34';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    const startX = Math.floor(minX / step) * step;
    const startY = Math.floor(minY / step) * step;
    for (let x = startX; x <= maxX; x += step) {
      const p = this.toCanvas(x, 0);
      ctx.moveTo(p.cx, 0);
      ctx.lineTo(p.cx, h);
    }
    for (let y = startY; y <= maxY; y += step) {
      const p = this.toCanvas(0, y);
      ctx.moveTo(0, p.cy);
      ctx.lineTo(w, p.cy);
    }
    ctx.stroke();

    // Origin axes.
    ctx.strokeStyle = '#444';
    ctx.lineWidth   = 1.2;
    const o = this.toCanvas(0, 0);
    ctx.beginPath();
    ctx.moveTo(0, o.cy); ctx.lineTo(w, o.cy);
    ctx.moveTo(o.cx, 0); ctx.lineTo(o.cx, h);
    ctx.stroke();

    // Origin label.
    ctx.fillStyle = '#666';
    ctx.font = '10px monospace';
    ctx.fillText('0,0', o.cx + 4, o.cy - 4);
  }

  private drawTrace(): void {
    if (!this.showTrace || this.tracePoints.length < 2) return;
    const ctx = this.ctx;
    ctx.beginPath();
    ctx.strokeStyle = '#7e57c2';
    ctx.lineWidth   = 1.5;
    for (let i = 0; i < this.tracePoints.length; i++) {
      const p = this.toCanvas(this.tracePoints[i][0], this.tracePoints[i][1]);
      if (i === 0) ctx.moveTo(p.cx, p.cy);
      else         ctx.lineTo(p.cx, p.cy);
    }
    ctx.stroke();
  }

  private drawWaypoint(wp: Waypoint): void {
    const ctx = this.ctx;
    const p = this.toCanvas(wp.pos[0], wp.pos[1]);

    const idx = this.waypoints.indexOf(wp);
    if (idx > 0) {
      const prev = this.waypoints[idx - 1];
      const pp = this.toCanvas(prev.pos[0], prev.pos[1]);
      ctx.strokeStyle = '#3a78c2';
      ctx.lineWidth   = 2;
      ctx.beginPath();
      ctx.moveTo(pp.cx, pp.cy);
      ctx.lineTo(p.cx, p.cy);
      ctx.stroke();
    }

    ctx.fillStyle = '#ffcc33';
    ctx.strokeStyle = '#000';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, WP_RADIUS, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#fff';
    ctx.font = '11px monospace';
    ctx.fillText(`${wp.id} (t=${wp.time.toFixed(1)}s)`, p.cx + WP_RADIUS + 4, p.cy - WP_RADIUS - 2);

    if (this.showVelocity) {
      const vx = wp.vel[0] * this.ppm;
      const vy = wp.vel[1] * this.ppm;
      const tip = { cx: p.cx + vx, cy: p.cy + vy };
      const speed = Math.hypot(vx, vy);

      ctx.strokeStyle = '#4caf50';
      ctx.fillStyle   = '#4caf50';
      ctx.lineWidth   = 2.5;
      ctx.beginPath();
      ctx.moveTo(p.cx, p.cy);
      ctx.lineTo(tip.cx, tip.cy);
      ctx.stroke();

      if (speed > 1) {
        const angle = Math.atan2(vy, vx);
        ctx.beginPath();
        ctx.moveTo(tip.cx, tip.cy);
        ctx.lineTo(
          tip.cx - VEL_HEAD_LEN * Math.cos(angle) + VEL_HEAD_WID * Math.sin(angle),
          tip.cy - VEL_HEAD_LEN * Math.sin(angle) - VEL_HEAD_WID * Math.cos(angle),
        );
        ctx.lineTo(
          tip.cx - VEL_HEAD_LEN * Math.cos(angle) - VEL_HEAD_WID * Math.sin(angle),
          tip.cy - VEL_HEAD_LEN * Math.sin(angle) + VEL_HEAD_WID * Math.cos(angle),
        );
        ctx.closePath();
        ctx.fill();
      }

      const sp = Math.hypot(wp.vel[0], wp.vel[1]);
      ctx.fillStyle = '#4caf50';
      ctx.font = '10px monospace';
      ctx.fillText(`v=${sp.toFixed(2)} m/s`, tip.cx + 4, tip.cy - 4);
    }
  }
}
