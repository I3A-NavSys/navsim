import {
  AfterViewInit,
  ChangeDetectionStrategy,
  Component,
  ElementRef,
  EventEmitter,
  Input,
  NgZone,
  OnChanges,
  OnDestroy,
  Output,
  SimpleChanges,
  ViewChild,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer.js';

import {
  AXIS_INDEX,
  Axis,
  AXIS_LABELS,
  FlightPlan,
  Vec3,
  Waypoint,
} from '../services/flight-plan.service';
import { PlanColorService } from '../services/plan-color.service';

/** Camera presets.
 *
 *  Coordinate convention:
 *    +X = right,  +Y = depth,  +Z = up.
 *    The "axis view" buttons (e.g. `−X`) position the camera
 *    perpendicular to that axis and look back at the world origin.
 *    `axis` records which world axis the camera is *parallel to*,
 *    used later to choose the matching `defaultX/Y/Z` when the user
 *    adds a waypoint with a click.
 */
interface CameraPreset {
  id: 'px' | 'nx' | 'py' | 'ny' | 'pz' | 'nz' | 'iso';
  label: string;
  pos: [number, number, number];
  up:   [number, number, number];
  axis: Axis | null;  // null for iso (free 3D placement)
}
/** Single colour shared by every velocity arrow in the viewer,
 *  regardless of plan — keeps the visual language consistent. */
const VIEWER_VELOCITY_ARROW_COLOR = 0xffd166;

const PRESETS: CameraPreset[] = [
  { id: 'iso', label: 'Iso',   pos: [ 60,  60,  60], up: [0, 0, 1], axis: null },
  { id: 'px',  label: '+X',    pos: [120,   0,   0], up: [0, 0, 1], axis: 'x' },
  { id: 'nx',  label: '−X',    pos: [-120, 0,   0], up: [0, 0, 1], axis: 'x' },
  { id: 'py',  label: '+Y',    pos: [0,    120,  0], up: [0, 0, 1], axis: 'y' },
  { id: 'ny',  label: '−Y',    pos: [0,   -120,  0], up: [0, 0, 1], axis: 'y' },
  { id: 'pz',  label: '+Z',    pos: [0,    0,  120], up: [0, 1, 0], axis: 'z' },
  { id: 'nz',  label: '−Z',    pos: [0,    0, -120], up: [0, 1, 0], axis: 'z' },
];

/** Which entity a drag operation is currently manipulating. */
type DragTarget =
  | { kind: 'none' }
  | { kind: 'wp-pos';   wpId: string }
  | { kind: 'wp-vel';   wpId: string };

/** Drag-mode the user picked from the toolbar. */
type DragMode = 'add' | 'move' | 'vel';

/**
 * 3D viewer (Three.js) with Z-up convention.
 *
 *  Renders every visible FlightPlan as:
 *    * a polyline (waypoint → waypoint)
 *    * waypoint spheres with id/time/velocity labels
 *    * velocity arrow `ArrowHelper`
 *    * the active simulation dot per plan
 *
 *  Pointer semantics:
 *    * Plain left click in empty canvas   → run the active tool
 *    * Shift + drag                       → orbit (OrbitControls only)
 *    * Ctrl  + drag                       → pan   (OrbitControls only)
 *    * Right click drag                    → pan   (OrbitControls)
 *
 *  Axis views (−X, +Y, …) lock OrbitControls — the user can only
 *  orbit / pan in the Iso view.
 */
@Component({
  selector: 'app-viewer3d',
  standalone: true,
  imports: [CommonModule, FormsModule],
  changeDetection: ChangeDetectionStrategy.OnPush,
  template: `
    <div class="viewer-shell">
      <canvas #canvas class="viewer-canvas"></canvas>
      <div #labels class="viewer-labels"></div>

      <div class="axis-bar">
        <ng-container *ngFor="let p of presets">
          <button class="axis-btn"
                  [class.active]="activeView === p.id"
                  (click)="setView(p.id)"
                  [title]="'Switch to ' + p.label + ' view'">
            {{ p.label }}
          </button>
        </ng-container>
        <span class="spacer"></span>
        <span class="legend">
          <span class="dot" [style.background]="toolColor()"></span>
          {{ toolLabel() }}
        </span>
      </div>

      <div class="tool-bar">
        <button class="tool-btn"
                [class.active]="tool === 'add'"
                (click)="setTool('add')"
                title="Right-click on empty canvas to add a waypoint">
          <span class="kbd">A</span>&nbsp;Add WP
        </button>
        <button class="tool-btn"
                [class.active]="tool === 'move'"
                (click)="setTool('move')"
                title="Left-drag a waypoint sphere to move it">
          <span class="kbd">M</span>&nbsp;Move WP
        </button>
        <button class="tool-btn"
                [class.active]="tool === 'vel'"
                (click)="setTool('vel')"
                title="Left-drag a velocity arrow tip to edit the vector">
          <span class="kbd">V</span>&nbsp;Edit velocity
        </button>

        <div class="defaults">
          <label>default X (m)
            <input class="input small"
                   type="number" step="1"
                   [value]="defaultX"
                   (change)="onDefaultAxisChange('x', $event)" />
          </label>
          <label>default Y (m)
            <input class="input small"
                   type="number" step="1"
                   [value]="defaultY"
                   (change)="onDefaultAxisChange('y', $event)" />
          </label>
          <label>default Z (m)
            <input class="input small"
                   type="number" step="1"
                   [value]="defaultZ"
                   (change)="onDefaultAxisChange('z', $event)" />
          </label>
        </div>

        <label class="check" (click)="$event.stopPropagation()">
          <input type="checkbox"
                 [checked]="showAxes"
                 (change)="toggleAxes($event)">
          Axes
        </label>
        <label class="check" (click)="$event.stopPropagation()">
          <input type="checkbox"
                 [checked]="showGrid"
                 (change)="toggleGrid($event)">
          Grid
        </label>
        <label class="check" (click)="$event.stopPropagation()">
          <input type="checkbox"
                 [checked]="showLabels"
                 (change)="toggleLabels($event)">
          Labels
        </label>

        <div class="label-fields">
          <small>label fields</small>
          <ng-container *ngFor="let f of labelFields">
            <button class="chip-btn"
                    [class.on]="labelFieldSet.has(f)"
                    (click)="toggleLabelField(f)">
              {{ f }}
            </button>
          </ng-container>
        </div>

        <div class="hint-line">
          Right-click = Add WP · Left-drag sphere = Move WP ·
          Left-drag arrow = Edit velocity · Wheel = Zoom ·
          Shift+drag = orbit · Ctrl+drag = pan.
        </div>
      </div>
    </div>
  `,
  styles: [`
    :host { display: block; width: 100%; height: 100%; min-height: 0; }
    .viewer-shell {
      position: relative;
      width: 100%;
      height: 100%;
      min-height: 0;
      background:
        radial-gradient(1200px 800px at 80% -10%, rgba(124, 92, 255, 0.12), transparent 60%),
        radial-gradient(900px 600px at -10% 110%, rgba(45, 212, 191, 0.08), transparent 60%),
        linear-gradient(180deg, #06080d, var(--bg-0));
      border-radius: var(--radius);
      overflow: hidden;
      border: 1px solid var(--border-soft);
    }
    .viewer-canvas, .viewer-labels {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      display: block;
    }
    .viewer-labels { pointer-events: none; }
    :host ::ng-deep .viewer-labels .wp-label {
      background: rgba(11, 13, 18, 0.85);
      border: 1px solid var(--border);
      color: var(--fg-0);
      padding: 4px 8px;
      border-radius: 8px;
      font-size: 11px;
      font-family: 'JetBrains Mono', ui-monospace, monospace;
      white-space: nowrap;
      box-shadow: var(--shadow-2);
      backdrop-filter: blur(8px);
    }
    :host ::ng-deep .viewer-labels .wp-label .id { color: var(--accent-2); font-weight: 600; }
    :host ::ng-deep .viewer-labels .wp-label .t  { color: var(--teal); }
    :host ::ng-deep .viewer-labels .wp-label .v  { color: var(--gold); }
    :host ::ng-deep .viewer-labels .wp-label .row { display: flex; gap: 8px; }
    :host ::ng-deep .viewer-labels .wp-label .row + .row { margin-top: 2px; }

    .axis-bar, .tool-bar {
      position: absolute;
      background: rgba(17, 20, 28, 0.95);
      backdrop-filter: blur(10px);
      border: 1px solid var(--border-soft);
      border-radius: 10px;
      padding: 4px;
      display: flex;
      gap: 2px;
      align-items: center;
      box-shadow: var(--shadow-2);
      z-index: 10;
    }
    .axis-bar {
      bottom: 12px;
      left: 50%;
      transform: translateX(-50%);
    }
    .tool-bar {
      top: 12px;
      left: 12px;
      flex-direction: column;
      align-items: stretch;
      width: 200px;
      gap: 4px;
    }
    .tool-bar .defaults {
      display: flex;
      flex-direction: column;
      gap: 2px;
      padding: 6px 0;
      border-top: 1px dashed var(--border-soft);
      border-bottom: 1px dashed var(--border-soft);
      margin: 4px 0;
    }
    .tool-bar .defaults label {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 4px;
      font-size: 10.5px;
      color: var(--fg-1);
    }
    .tool-bar .input.small { width: 76px; padding: 2px 6px; }
    .tool-bar .hint-line {
      font-size: 10px;
      color: var(--fg-3);
      text-align: left;
      padding-top: 4px;
      border-top: 1px dashed var(--border-soft);
      margin-top: 4px;
      line-height: 1.4;
    }
    .tool-bar .label-fields {
      display: flex;
      flex-wrap: wrap;
      gap: 4px;
      padding: 4px 0;
      border-top: 1px dashed var(--border-soft);
    }
    .tool-bar .label-fields small {
      flex-basis: 100%;
      font-size: 10px;
      color: var(--fg-3);
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }
    .chip-btn {
      background: var(--bg-2);
      border: 1px solid var(--border);
      color: var(--fg-1);
      padding: 3px 8px;
      border-radius: 999px;
      cursor: pointer;
      font-size: 10.5px;
      font-family: 'JetBrains Mono', ui-monospace, monospace;
    }
    .chip-btn.on {
      background: var(--accent-soft);
      border-color: rgba(124, 92, 255, 0.4);
      color: var(--accent-2);
    }
    .axis-btn, .tool-btn {
      background: transparent;
      border: 1px solid transparent;
      color: var(--fg-1);
      padding: 4px 10px;
      border-radius: 6px;
      cursor: pointer;
      font-size: 12px;
      font-weight: 500;
      display: flex;
      align-items: center;
      gap: 4px;
      transition: background var(--t-fast), border-color var(--t-fast);
    }
    .axis-btn:hover, .tool-btn:hover { background: var(--bg-3); color: var(--fg-0); }
    .axis-btn.active {
      background: var(--accent-soft);
      border-color: rgba(124, 92, 255, 0.35);
      color: var(--accent-2);
    }
    .tool-btn.active {
      background: var(--accent);
      color: #fff;
    }
    .spacer { flex: 1; }
    .check {
      display: flex;
      align-items: center;
      gap: 4px;
      font-size: 11px;
      color: var(--fg-1);
      padding: 2px 6px;
    }
    .check input { accent-color: var(--accent); }
    .legend {
      display: flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      font-size: 11px;
      color: var(--fg-1);
    }
    :host ::ng-deep .axis-bar { z-index: 12; }
  `],
})
export class Viewer3dComponent implements AfterViewInit, OnChanges, OnDestroy {
  @Input() plans: FlightPlan[] = [];
  @Input() simPoints: Map<string, Vec3> = new Map();
  @Input() defaultX = 0;
  @Input() defaultY = 0;
  @Input() defaultZ = 50;

  /** Emitted add / move / velocity events now carry `planId` so
   *  the application can dispatch the mutation to the right plan
   *  even when the affected waypoint belongs to a plan that is
   *  not the user's currently-selected one. */
  @Output() addWpRequested = new EventEmitter<{ planId?: string; x: number; y: number; z: number }>();
  @Output() wpMoved       = new EventEmitter<{ planId: string; id: string; pos: Vec3 }>();
  @Output() velChanged    = new EventEmitter<{ planId: string; id: string; vel: Vec3 }>();

  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;
  @ViewChild('labels', { static: true })  labelsRef!: ElementRef<HTMLDivElement>;

  readonly presets = PRESETS;
  activeView: CameraPreset['id'] = 'iso';

  tool: DragMode = 'add';
  showAxes = true;
  showGrid = true;
  /** `false` to hide every CSS2D label globally. */
  showLabels = true;
  /** Which fields each waypoint label shows. */
  readonly labelFields = ['id', 'time', 'pos', 'vel'] as const;
  labelFieldSet: Set<typeof this.labelFields[number]> = new Set(['id', 'time', 'vel']);

  // ----------------------------------------------------------------
  //  Three.js plumbing
  // ----------------------------------------------------------------
  private renderer!: THREE.WebGLRenderer;
  private labelRenderer!: CSS2DRenderer;
  private scene!: THREE.Scene;
  private camera!: THREE.PerspectiveCamera;
  private controls!: OrbitControls;
  private clock = new THREE.Clock();
  private rafId = 0;
  private resizeObserver!: ResizeObserver;
  private raycaster = new THREE.Raycaster();
  private pointer = new THREE.Vector2();
  private dragPlane: THREE.Plane | null = null;
  private dragTarget: DragTarget = { kind: 'none' };

  private plansGroup   = new THREE.Group();
  private simGroup     = new THREE.Group();
  private wpMeshes     = new Map<string, Map<string, THREE.Mesh>>();
  private wpArrows     = new Map<string, Map<string, THREE.ArrowHelper>>();
  private planLines    = new Map<string, THREE.Line>();
  private wpToPlan     = new Map<string, string>();
  private currentWPs   = new Map<string, Waypoint>();
  private planRoots    = new Map<string, THREE.Object3D>();
  /** Per-plan simulation dots — recreated lazily so we can mutate
   *  their positions in place during playback. */
  private simDots: Map<string, { dot: THREE.Mesh; wire: THREE.Mesh }> = new Map();
  /** Hover state for the picked-velocity-arrow, drives cursor. */
  private hoveringArrow = false;
  /** Hover state for the picked-waypoint-sphere. */
  private hoveringWp    = false;

  constructor(
    private zone: NgZone,
    private colors: PlanColorService,
  ) {}

  ngAfterViewInit(): void {
    this.zone.runOutsideAngular(() => this.initThree());
    this.applyPlans();
    // Wire the OrbitControls state to whatever tool is active at
    // startup — by default the Add tool disables the camera.
    this.setTool(this.tool);
  }

  ngOnChanges(changes: SimpleChanges): void {
    if (!this.scene) return;
    if (changes['plans']) this.applyPlans();
    if (changes['simPoints']) this.applySim();
  }

  ngOnDestroy(): void {
    cancelAnimationFrame(this.rafId);
    this.resizeObserver?.disconnect();
    this.renderer?.dispose();
    this.cleanupAllMeshes();
  }

  // ----------------------------------------------------------------
  //  Public UI hooks
  // ----------------------------------------------------------------
  setView(id: CameraPreset['id']): void {
    this.activeView = id;
    const p = PRESETS.find((pp) => pp.id === id);
    if (!p) return;
    this.applyCameraPreset(p);
    // Axis views lock orbit — the camera can still pan and zoom,
    // so the user can frame the scene in any view. Iso enables
    // everything.
    const isIso = id === 'iso';
    this.controls.enableRotate = isIso;
    this.controls.enablePan    = true;
    this.controls.enableZoom   = true;   // always enabled
    if (this.tool !== 'add') this.controls.enabled = true;
  }

  setTool(t: DragMode): void {
    this.tool = t;
    // OrbitControls stays active across ALL tools — the user can
    // still orbit / pan / zoom regardless of which drag mode is
    // selected. The right mouse button is reserved for the "add
    // waypoint" gesture in 'add' mode (handled by the viewer's
    // own onPointerDown), so we detach it from OrbitControls in
    // that mode to stop the camera from also panning.
    this.controls.enabled = true;
    if (this.controls && this.controls.mouseButtons) {
      // Any value other than a real MOUSE.* enum disables the
      // button inside OrbitControls — null is the documented
      // "off" marker in newer three.js typings.
      this.controls.mouseButtons.RIGHT = (t === 'add') ? null : THREE.MOUSE.PAN;
    }
  }

  toggleAxes(ev: Event): void {
    this.showAxes = (ev.target as HTMLInputElement).checked;
    this.syncHelpers();
  }

  toggleGrid(ev: Event): void {
    this.showGrid = (ev.target as HTMLInputElement).checked;
    this.syncHelpers();
  }

  toggleLabels(ev: Event): void {
    this.showLabels = (ev.target as HTMLInputElement).checked;
    // Hide the labels in DOM (set `label.visible = false`); the
    // velocity arrows stay visible regardless.
    for (const wpmap of this.wpMeshes.values()) {
      for (const mesh of wpmap.values()) {
        const lbl = mesh.userData['label'] as CSS2DObject | undefined;
        if (lbl) lbl.visible = this.showLabels;
      }
    }
  }

  toggleLabelField(f: typeof this.labelFields[number]): void {
    if (this.labelFieldSet.has(f)) this.labelFieldSet.delete(f);
    else this.labelFieldSet.add(f);
    // Just regenerate the HTML of every label — the velocity
    // arrows are not affected by label-field toggles.
    this.refreshAllLabels();
  }

  /** Rebuild the HTML of every waypoint label to reflect the
   *  current `showLabels` + `labelFieldSet` configuration. */
  private refreshAllLabels(): void {
    for (const [planId, wpmap] of this.wpMeshes) {
      for (const [wpId, mesh] of wpmap) {
        const label = mesh.userData['label'] as CSS2DObject | undefined;
        if (!label) continue;
        label.visible = this.showLabels;
        if (!this.showLabels) continue;
        const wp = this.plans.find((p) => p.id === planId)?.waypoints.find((w) => w.id === wpId);
        if (!wp) continue;
        label.element.innerHTML = this.buildLabelHtml(wp);
      }
    }
  }

  /** Build the inner HTML for one waypoint's CSS2D label. The
   *  order matches `labelFields`. Velocity is shown as a scalar
   *  magnitude (m/s) — the per-axis breakdown is in the 3D arrow. */
  private buildLabelHtml(wp: Waypoint): string {
    const rows: string[] = [];
    for (const f of this.labelFields) {
      if (!this.labelFieldSet.has(f)) continue;
      if (f === 'id') {
        rows.push(`<div class="row"><span class="id">${escapeHtml(wp.id)}</span></div>`);
      } else if (f === 'time') {
        rows.push(`<div class="row"><span class="t">t=${wp.time.toFixed(2)}s</span></div>`);
      } else if (f === 'pos') {
        rows.push(`<div class="row"><span class="v">p=(${wp.pos[0].toFixed(1)}, ${wp.pos[1].toFixed(1)}, ${wp.pos[2].toFixed(1)})</span></div>`);
      } else if (f === 'vel') {
        const speed = Math.hypot(wp.vel[0], wp.vel[1], wp.vel[2]);
        rows.push(`<div class="row"><span class="v">|v|=${speed.toFixed(2)} m/s</span></div>`);
      }
    }
    return rows.join('');
  }

  onDefaultAxisChange(axis: Axis, ev: Event): void {
    const v = parseFloat((ev.target as HTMLInputElement).value);
    if (!Number.isFinite(v)) return;
    if (axis === 'x') this.defaultX = v;
    else if (axis === 'y') this.defaultY = v;
    else this.defaultZ = v;
  }

  toolLabel(): string {
    switch (this.tool) {
      case 'add':  return 'Add WP — click empty canvas';
      case 'move': return 'Move WP — drag a sphere';
      case 'vel':  return 'Edit velocity — drag an arrow';
    }
  }

  toolColor(): string {
    switch (this.tool) {
      case 'add':  return '#7c5cff';
      case 'move': return '#2dd4bf';
      case 'vel':  return '#fbbf24';
    }
  }

  // ----------------------------------------------------------------
  //  Three.js bootstrap
  // ----------------------------------------------------------------
  private initThree(): void {
    const canvas = this.canvasRef.nativeElement;
    const labelsHost = this.labelsRef.nativeElement;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x000000, 0);

    this.labelRenderer = new CSS2DRenderer({ element: labelsHost });
    this.labelRenderer.setSize(canvas.clientWidth, canvas.clientHeight);

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xb0b7c5, 0.55));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(80, 60, 100);
    this.scene.add(dir);
    this.scene.add(this.plansGroup);
    this.scene.add(this.simGroup);

    const aspect = canvas.clientWidth / Math.max(canvas.clientHeight, 1);
    this.camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 5000);
    // Z-up convention: the upward axis in world coordinates is +Z.
    this.camera.up.set(0, 0, 1);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0, 0);
    this.controls.mouseButtons = {
      LEFT:   THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT:  THREE.MOUSE.PAN,
    };
    this.controls.enablePan       = true;
    this.controls.enableRotate    = true;
    this.controls.screenSpacePanning = true;

    this.applyCameraPreset(PRESETS[0]);
    this.controls.enabled = true;  // iso view starts enabled

    canvas.addEventListener('pointerdown',   this.onPointerDown);
    canvas.addEventListener('pointermove',   this.onPointerMove);
    canvas.addEventListener('pointerup',     this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);
    canvas.addEventListener('pointermove',   this.onHover);
    // Right-click is reserved for "add waypoint" — block the
    // browser context menu so it doesn't pop up over our tooltip.
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    window.addEventListener('keydown',       this.onKeyDown);

    this.resizeObserver = new ResizeObserver(() => this.onResize());
    this.resizeObserver.observe(canvas);

    const animate = () => {
      this.rafId = requestAnimationFrame(animate);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.labelRenderer.render(this.scene, this.camera);
      this.clock.getDelta();
    };
    animate();
  }

  private applyCameraPreset(p: CameraPreset): void {
    if (!this.camera) return;
    this.controls.target.set(0, 0, 0);
    this.camera.up.set(p.up[0], p.up[1], p.up[2]);
    this.camera.position.set(p.pos[0], p.pos[1], p.pos[2]);
    this.camera.lookAt(this.controls.target);
    this.controls.update();
  }

  private onResize(): void {
    const canvas = this.canvasRef.nativeElement;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    this.renderer.setSize(w, h, false);
    this.labelRenderer.setSize(w, h);
    if ((this.camera as any).isPerspectiveCamera) {
      (this.camera as THREE.PerspectiveCamera).aspect = w / Math.max(h, 1);
      (this.camera as THREE.PerspectiveCamera).updateProjectionMatrix();
    }
  }

  // ----------------------------------------------------------------
  //  Plans / waypoints sync
  // ----------------------------------------------------------------
  private applyPlans(): void {
    if (!this.scene) return;

    const ids = new Set(this.plans.map((p) => p.id));

    // Drop plans that no longer exist (deleted or filtered out).
    for (const [id, root] of this.planRoots) {
      if (!ids.has(id)) {
        this.plansGroup.remove(root);
        this.cleanupSubtree(root);
        this.planRoots.delete(id);
        this.planLines.delete(id);
        this.wpMeshes.delete(id);
        this.wpArrows.delete(id);
      }
    }

    this.currentWPs.clear();
    this.wpToPlan.clear();

    this.plans.forEach((plan) => {
      // Stable per-plan colour via the shared `PlanColorService` —
      // guarantees the trace, dots, and plan-tab share the same hue.
      const colorHex = this.colors.colorFor(plan.id);
      const color = new THREE.Color(colorHex);

      let root = this.planRoots.get(plan.id);
      if (!root) {
        root = new THREE.Group();
        this.plansGroup.add(root);
        this.planRoots.set(plan.id, root);
        this.wpMeshes.set(plan.id, new Map());
        this.wpArrows.set(plan.id, new Map());
      }
      root.visible = plan.visible;

      const wpMap    = this.wpMeshes.get(plan.id)!;
      const arrowMap = this.wpArrows.get(plan.id)!;

      const keptIds = new Set<string>();
      for (const wp of plan.waypoints) {
        keptIds.add(wp.id);
        this.currentWPs.set(wp.id, wp);
        this.wpToPlan.set(wp.id, plan.id);
      }

      // ── Trace curve ────────────────────────────────────────
      // Always render the C⁵ trace returned by the backend, not a
      // straight line between waypoints. The trace array is the
      // actual kinematic path the simulation will follow.
      const tr = plan.trace;
      const oldLine = this.planLines.get(plan.id);
      if (oldLine) {
        root.remove(oldLine);
        oldLine.geometry.dispose();
        (oldLine.material as THREE.Material).dispose();
      }
      if (tr.t && tr.t.length > 1) {
        const tracePos = new Float32Array(tr.t.length * 3);
        for (let i = 0; i < tr.t.length; i++) {
          tracePos[i * 3]     = tr.pos[i][0];
          tracePos[i * 3 + 1] = tr.pos[i][1];
          tracePos[i * 3 + 2] = tr.pos[i][2];
        }
        const lineGeo = new THREE.BufferGeometry();
        lineGeo.setAttribute('position', new THREE.BufferAttribute(tracePos, 3));
        const lineMat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
        const line = new THREE.Line(lineGeo, lineMat);
        line.renderOrder = 0;
        root.add(line);
        this.planLines.set(plan.id, line);
      } else {
        this.planLines.delete(plan.id);
      }

      // Per-waypoint spheres + labels + velocity arrows.
      for (const wp of plan.waypoints) {
        let mesh = wpMap.get(wp.id);
        if (!mesh) {
          const sphere = new THREE.SphereGeometry(1.2, 18, 12);
          const m = new THREE.MeshBasicMaterial({ color });
          mesh = new THREE.Mesh(sphere, m);
          mesh.userData['wpId']   = wp.id;
          mesh.userData['planId'] = plan.id;
          root.add(mesh);
          wpMap.set(wp.id, mesh);

          const label = this.buildLabel();
          label.userData['wpId'] = wp.id;
          root.add(label);
          mesh.userData['label'] = label;
        }
        mesh.position.set(wp.pos[0], wp.pos[1], wp.pos[2]);
        (mesh.material as THREE.MeshBasicMaterial).color.copy(color);

        const label = mesh.userData['label'] as CSS2DObject;
        label.position.set(wp.pos[0], wp.pos[1], wp.pos[2] + 3);
        label.visible = this.showLabels;
        label.element.innerHTML = this.buildLabelHtml(wp);
        // Velocity arrow is always visible (independent of the
        // label-fields toggle — the user wants the arrow to be a
        // permanent part of every waypoint, like the sphere itself).
        let arrow = arrowMap.get(wp.id);
        if (!arrow) {
          arrow = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 0, 0),
            1, VIEWER_VELOCITY_ARROW_COLOR, 2, 1.4,
          );
          arrow.userData['wpId']   = wp.id;
          arrow.userData['planId'] = plan.id;
          root.add(arrow);
          arrowMap.set(wp.id, arrow);
        }
        if (arrow) {
          arrow.visible = true;
          const v = new THREE.Vector3(wp.vel[0], wp.vel[1], wp.vel[2]);
          const len = v.length();
          if (len > 0.001) {
            arrow.setDirection(v.clone().normalize());
            arrow.setLength(
              Math.min(60, len + 5),
              Math.min(8, 0.3 * Math.min(60, len + 5) + 1),
              Math.min(8, 0.2 * Math.min(60, len + 5) + 1),
            );
            arrow.position.set(wp.pos[0], wp.pos[1], wp.pos[2]);
            arrow.setColor(new THREE.Color(VIEWER_VELOCITY_ARROW_COLOR));
          } else {
            // Collapse the arrow to a near-zero length when the
            // waypoint has no velocity yet — it still occupies its
            // slot in `arrowMap` so picking remains consistent.
            arrow.setLength(0.0001, 0, 0);
          }
        }
      }

      // Drop stale meshes / arrows + their labels.
      for (const [id, m] of wpMap) {
        if (!keptIds.has(id)) {
          root.remove(m);
          const lbl = m.userData['label'] as CSS2DObject | undefined;
          if (lbl) root.remove(lbl);
          m.geometry.dispose();
          (m.material as THREE.Material).dispose();
          wpMap.delete(id);
        }
      }
      for (const [id, a] of arrowMap) {
        if (!keptIds.has(id)) {
          root.remove(a);
          arrowMap.delete(id);
        }
      }
    });

    this.syncHelpers();
  }

  private applySim(): void {
    // Reuse existing dot meshes (per planId) so we can update their
    // positions in place each frame instead of recreating them.
    const liveIds = new Set(this.simPoints.keys());
    // Drop stale dots.
    for (const [planId, pair] of this.simDots) {
      if (!liveIds.has(planId)) {
        this.simGroup.remove(pair.dot);
        this.simGroup.remove(pair.wire);
        pair.dot.geometry.dispose();
        (pair.dot.material as THREE.Material).dispose();
        pair.wire.geometry.dispose();
        (pair.wire.material as THREE.Material).dispose();
        this.simDots.delete(planId);
      }
    }
    for (const [planId, pos] of this.simPoints) {
      let pair = this.simDots.get(planId);
      if (!pair) {
        const sphere = new THREE.SphereGeometry(2.2, 24, 18);
        const m = new THREE.MeshStandardMaterial({
          color: 0xffd166,
          emissive: 0xffd166,
          emissiveIntensity: 0.9,
          metalness: 0.05,
          roughness: 0.35,
        });
        const dot = new THREE.Mesh(sphere, m);
        this.simGroup.add(dot);

        const edges = new THREE.SphereGeometry(2.4, 12, 8);
        const edgeMat = new THREE.MeshBasicMaterial({
          color: 0xffffff, wireframe: true, transparent: true, opacity: 0.55,
        });
        const wire = new THREE.Mesh(edges, edgeMat);
        this.simGroup.add(wire);

        pair = { dot, wire };
        this.simDots.set(planId, pair);
      }
      pair.dot.position.set(pos[0], pos[1], pos[2]);
      pair.wire.position.set(pos[0], pos[1], pos[2]);
    }
  }

  /** Live update of a single plan's trace geometry without
   *  re-building everything. The mutation stream still drives the
   *  authoritative geometry via `applyPlans`, but during a drag
   *  we want to see the trace follow the cursor in real time. */
  private refreshPlanTrace(planId: string): void {
    const plan = this.plans.find((p) => p.id === planId);
    if (!plan) return;
    const line = this.planLines.get(planId);
    if (!line) return;
    const geo = line.geometry as THREE.BufferGeometry;
    const tr = plan.trace;
    if (!tr.t || tr.t.length < 2) return;
    const arr = geo.attributes['position'].array as Float32Array;
    if (arr.length !== tr.t.length * 3) {
      // Length changed (server added samples) — let applyPlans
      // handle the rebuild next time it runs.
      this.applyPlans();
      return;
    }
    for (let i = 0; i < tr.t.length; i++) {
      arr[i * 3]     = tr.pos[i][0];
      arr[i * 3 + 1] = tr.pos[i][1];
      arr[i * 3 + 2] = tr.pos[i][2];
    }
    geo.attributes['position'].needsUpdate = true;
    geo.computeBoundingSphere();
  }

  /** Adjust the trace line in real time while the user drags a WP
   *  or its velocity. We can't recompute the C⁵ curve locally
   *  without the higher-order derivatives, so we re-derive only
   *  the segments touching the affected WP using linear blends of
   *  the live local cache — accurate enough at the time-scale of
   *  a drag (~60 Hz) and avoids re-allocating the 1000-sample
   *  buffer each frame. */
  private liveAdjustTrace(planId: string, draggedWpId: string): void {
    const line = this.planLines.get(planId);
    if (!line) return;
    const geo = line.geometry as THREE.BufferGeometry;
    const arr = geo.attributes['position'].array as Float32Array;
    const plan = this.plans.find((p) => p.id === planId);
    if (!plan) return;
    const tr = plan.trace;
    if (!tr.t || tr.t.length < 2) return;

    const idx = plan.waypoints.findIndex((w) => w.id === draggedWpId);
    if (idx < 0) return;
    const dragged = this.currentWPs.get(draggedWpId);
    if (!dragged) return;

    // Re-derive the segment between [idx-1, idx] (use linear blend
    // of the previous WP and the live dragged WP) and [idx, idx+1].
    const n = tr.t.length;
    for (let i = 0; i < n; i++) {
      const t = tr.t[i];
      if (t < dragged.time) {
        // Before the dragged WP — blend the previous waypoint
        // with the live position of the dragged WP if we are past
        // it, otherwise keep the server trace.
        const prev = idx > 0 ? plan.waypoints[idx - 1] : null;
        if (prev && t >= prev.time) {
          const u = (t - prev.time) / Math.max(1e-9, dragged.time - prev.time);
          const uClamped = Math.max(0, Math.min(1, u));
          const px = prev.pos[0] + (dragged.pos[0] - prev.pos[0]) * uClamped;
          const py = prev.pos[1] + (dragged.pos[1] - prev.pos[1]) * uClamped;
          const pz = prev.pos[2] + (dragged.pos[2] - prev.pos[2]) * uClamped;
          arr[i * 3]     = px;
          arr[i * 3 + 1] = py;
          arr[i * 3 + 2] = pz;
        }
        // Otherwise keep the server trace value.
      } else if (t <= dragged.time + 1e-9) {
        arr[i * 3]     = dragged.pos[0];
        arr[i * 3 + 1] = dragged.pos[1];
        arr[i * 3 + 2] = dragged.pos[2];
      } else {
        // After the dragged WP — blend the live dragged WP with
        // the next waypoint.
        const next = idx + 1 < plan.waypoints.length ? plan.waypoints[idx + 1] : null;
        if (next && t <= next.time) {
          const u = (t - dragged.time) / Math.max(1e-9, next.time - dragged.time);
          const uClamped = Math.max(0, Math.min(1, u));
          const px = dragged.pos[0] + (next.pos[0] - dragged.pos[0]) * uClamped;
          const py = dragged.pos[1] + (next.pos[1] - dragged.pos[1]) * uClamped;
          const pz = dragged.pos[2] + (next.pos[2] - dragged.pos[2]) * uClamped;
          arr[i * 3]     = px;
          arr[i * 3 + 1] = py;
          arr[i * 3 + 2] = pz;
        }
      }
    }
    geo.attributes['position'].needsUpdate = true;
    geo.computeBoundingSphere();
  }

  /** Update a single waypoint's position in the local cache and
   *  re-render it on screen. Used during drags for instant
   *  visual feedback before the server response arrives. */
  private updateWpVisual(planId: string, wpId: string, pos: [number, number, number], vel?: [number, number, number]): void {
    const wp = this.currentWPs.get(wpId);
    if (!wp) return;
    const newWp = vel ? { ...wp, pos, vel } : { ...wp, pos };
    this.currentWPs.set(wpId, newWp);

    const mesh = this.findWpMesh(wpId);
    if (mesh) {
      mesh.position.set(pos[0], pos[1], pos[2]);
      const label = mesh.userData['label'] as CSS2DObject | undefined;
      if (label) label.position.set(pos[0], pos[1], pos[2] + 3);
      if (label) label.element.innerHTML = this.buildLabelHtml(newWp);
    }

    // Arrow follows the WP.
    const arrow = this.wpArrows.get(planId)?.get(wpId);
    if (arrow) {
      arrow.position.set(pos[0], pos[1], pos[2]);
      const v = new THREE.Vector3(newWp.vel[0], newWp.vel[1], newWp.vel[2]);
      const len = v.length();
      if (len > 0.001) {
        arrow.setDirection(v.clone().normalize());
        arrow.setLength(
          Math.min(60, len + 5),
          Math.min(8, 0.3 * Math.min(60, len + 5) + 1),
          Math.min(8, 0.2 * Math.min(60, len + 5) + 1),
        );
      } else {
        arrow.setLength(0.0001, 0, 0);
      }
    }
  }

  private syncHelpers(): void {
    // Drop existing helper objects (by name) — idempotent.
    const stale = this.scene.children.filter(
      (c) => c.name === 'axes-helper' || c.name === 'grid-helper',
    );
    for (const c of stale) {
      this.scene.remove(c);
      const anyC = c as unknown as { geometry?: { dispose(): void }; material?: { dispose(): void } | { dispose(): void }[] };
      anyC.geometry?.dispose?.();
      const m = anyC.material;
      if (Array.isArray(m)) m.forEach((mm) => mm.dispose?.());
      else m?.dispose?.();
    }

    if (this.showAxes) {
      const ax = new THREE.AxesHelper(40);
      ax.name = 'axes-helper';
      this.scene.add(ax);
    }
    if (this.showGrid) {
      const g = new THREE.GridHelper(160, 16, 0x4b556e, 0x232a3b);
      g.name = 'grid-helper';
      // GridHelper lies on the X/Y plane by default — but our
      // world is Z-up, so rotate it onto X/Y (the floor plane in
      // NED coordinates). Z remains the up axis.
      g.rotation.x = Math.PI / 2;
      this.scene.add(g);
    }
  }

  private buildLabel(): CSS2DObject {
    const el = document.createElement('div');
    el.classList.add('wp-label');
    const obj = new CSS2DObject(el);
    obj.center.set(0.5, 1);
    return obj;
  }

  private cleanupAllMeshes(): void {
    for (const root of this.planRoots.values()) this.cleanupSubtree(root);
    this.planRoots.clear();
    this.wpMeshes.clear();
    this.wpArrows.clear();
    this.planLines.clear();
    this.currentWPs.clear();
    this.wpToPlan.clear();
  }

  private cleanupSubtree(root: THREE.Object3D): void {
    // CSS2DObjects (waypoint labels) need their DOM element
    // detached when the underlying object is removed from the
    // scene, otherwise CSS2DRenderer leaves them attached to the
    // host. Walk the subtree once and detach each.
    root.traverse((o) => {
      const obj = o as unknown as THREE.Object3D & {
        isCSS2DObject?: boolean;
        element?: { parentNode?: Node | null };
      };
      if (obj.isCSS2DObject && obj.element?.parentNode) {
        obj.element.parentNode.removeChild(obj.element as unknown as Node);
      }
      const m = o as unknown as {
        geometry?: { dispose(): void };
        material?: { dispose(): void } | { dispose(): void }[];
      };
      m.geometry?.dispose?.();
      const mat = m.material;
      if (Array.isArray(mat)) mat.forEach((mm) => mm.dispose?.());
      else mat?.dispose?.();
    });
  }

  // ----------------------------------------------------------------
  //  Picking
  // ----------------------------------------------------------------
  private toPointer(ev: PointerEvent | MouseEvent): void {
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    this.pointer.x =  ((ev.clientX - rect.left) / rect.width)  * 2 - 1;
    this.pointer.y = -((ev.clientY - rect.top)  / rect.height) * 2 + 1;
  }

  private intersectWp(): { wpId: string; planId: string; point: THREE.Vector3 } | null {
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const meshes: THREE.Object3D[] = [];
    for (const m of this.wpMeshes.values()) for (const mm of m.values()) meshes.push(mm);
    const hit = this.raycaster.intersectObjects(meshes, false)[0];
    if (!hit) return null;
    return {
      wpId: hit.object.userData['wpId'] as string,
      planId: hit.object.userData['planId'] as string,
      point: hit.point,
    };
  }

  private intersectArrow(): { wpId: string; planId: string } | null {
    this.raycaster.setFromCamera(this.pointer, this.camera);
    // Build pick-proxies: a big sphere at each arrow tip + spheres
    // along the shaft. WPs are picked even when their velocity is
    // zero (the first drag creates the velocity vector).
    const proxies: { obj: THREE.Object3D; sphere: THREE.Sphere }[] = [];
    for (const [wpId, wp] of this.currentWPs) {
      const planId = this.wpToPlan.get(wpId);
      if (!planId) continue;

      const v = wp.vel;
      const speed = Math.hypot(v[0], v[1], v[2]);

      if (speed > 0.05) {
        // Tip of the existing arrow.
        const tip: [number, number, number] = [
          wp.pos[0] + v[0], wp.pos[1] + v[1], wp.pos[2] + v[2],
        ];
        proxies.push({
          obj: this.makeProxy(wpId, planId),
          sphere: new THREE.Sphere(new THREE.Vector3(tip[0], tip[1], tip[2]), 4),
        });
        // Along the shaft.
        for (let i = 1; i < 4; i++) {
          const f = i / 4;
          const p: [number, number, number] = [
            wp.pos[0] + v[0] * f, wp.pos[1] + v[1] * f, wp.pos[2] + v[2] * f,
          ];
          proxies.push({
            obj: this.makeProxy(wpId, planId),
            sphere: new THREE.Sphere(new THREE.Vector3(p[0], p[1], p[2]), 2.5),
          });
        }
      } else {
        // No velocity yet — fall back to a sphere at the WP itself
        // so the user can "grow" a velocity vector out of nothing.
        proxies.push({
          obj: this.makeProxy(wpId, planId),
          sphere: new THREE.Sphere(new THREE.Vector3(wp.pos[0], wp.pos[1], wp.pos[2]), 4),
        });
      }
    }
    if (!proxies.length) return null;
    let best: { d: number; obj: THREE.Object3D } | null = null;
    const out = new THREE.Vector3();
    for (const p of proxies) {
      if (this.raycaster.ray.intersectSphere(p.sphere, out)) {
        const d = this.raycaster.ray.origin.distanceTo(out);
        if (!best || d < best.d) best = { d, obj: p.obj };
      }
    }
    if (!best) return null;
    return {
      wpId: best.obj.userData['wpId'] as string,
      planId: best.obj.userData['planId'] as string,
    };
  }

  private makeProxy(wpId: string, planId: string): THREE.Object3D {
    const obj = new THREE.Object3D();
    obj.userData['wpId']   = wpId;
    obj.userData['planId'] = planId;
    return obj;
  }

  // ----------------------------------------------------------------
  //  Pointer / drag handlers
  // ----------------------------------------------------------------
  private onPointerDown = (ev: PointerEvent): void => {
    this.toPointer(ev);

    // ── Right-click is reserved for "add waypoint". ────────────
    if (ev.button === 2 && this.tool === 'add') {
      this.startAddAtCursor();
      return;
    }
    // Anything outside the add tool (and any left/middle click)
    // falls through to OrbitControls or to the picking code below.

    const isLeft = ev.button === 0;
    if (!isLeft) return;

    // Shift + drag = orbit, Ctrl + drag = pan. Let OrbitControls
    // handle it; don't do anything tool-specific.
    if (ev.shiftKey || ev.ctrlKey) return;

    // Tool-specific behaviour (left button only).
    if (this.tool === 'move') {
      const hit = this.intersectWp();
      if (hit) this.startDrag({ kind: 'wp-pos', wpId: hit.wpId }, ev);
      return;
    }
    if (this.tool === 'vel') {
      const arrowHit = this.intersectArrow();
      if (arrowHit) this.startDrag({ kind: 'wp-vel', wpId: arrowHit.wpId }, ev);
      return;
    }
    // 'add' tool: only right-click handles it; left-click is a no-op.
  };

  private onPointerMove = (ev: PointerEvent): void => {
    this.toPointer(ev);
    if (this.dragTarget.kind === 'none' || !this.dragPlane) return;
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const out = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.dragPlane, out)) return;

    if (this.dragTarget.kind === 'wp-pos') {
      const wp = this.currentWPs.get(this.dragTarget.wpId);
      if (!wp) return;
      const newPos: [number, number, number] = [+out.x.toFixed(3), +out.y.toFixed(3), +out.z.toFixed(3)];
      const planId = this.wpToPlan.get(this.dragTarget.wpId);
      if (!planId) return;
      // Local visual update — instant feedback, no server round-trip.
      this.updateWpVisual(planId, this.dragTarget.wpId, newPos);
      this.liveAdjustTrace(planId, this.dragTarget.wpId);
      this.wpMoved.emit({ planId, id: this.dragTarget.wpId, pos: newPos });
    } else if (this.dragTarget.kind === 'wp-vel') {
      const wp = this.currentWPs.get(this.dragTarget.wpId);
      if (!wp) return;
      const origin = new THREE.Vector3(wp.pos[0], wp.pos[1], wp.pos[2]);
      const v = out.clone().sub(origin);
      const newVel: [number, number, number] = [+v.x.toFixed(3), +v.y.toFixed(3), +v.z.toFixed(3)];
      const planId = this.wpToPlan.get(this.dragTarget.wpId);
      if (!planId) return;
      // Local visual update of the arrow tip — instant feedback.
      const arrow = this.wpArrows.get(planId)?.get(this.dragTarget.wpId);
      if (arrow) {
        arrow.position.set(wp.pos[0], wp.pos[1], wp.pos[2]);
        const vDir = new THREE.Vector3(newVel[0], newVel[1], newVel[2]);
        const len = vDir.length();
        if (len > 0.001) {
          arrow.setDirection(vDir.clone().normalize());
          arrow.setLength(
            Math.min(60, len + 5),
            Math.min(8, 0.3 * Math.min(60, len + 5) + 1),
            Math.min(8, 0.2 * Math.min(60, len + 5) + 1),
          );
        } else {
          arrow.setLength(0.0001, 0, 0);
        }
      }
      // Update the cached velocity so the label shows the new vector.
      this.currentWPs.set(this.dragTarget.wpId, { ...wp, vel: newVel });
      const mesh = this.findWpMesh(this.dragTarget.wpId);
      if (mesh) {
        const label = mesh.userData['label'] as CSS2DObject | undefined;
        if (label) {
          label.element.innerHTML = this.buildLabelHtml({ ...wp, vel: newVel });
        }
      }
      this.velChanged.emit({ planId, id: this.dragTarget.wpId, vel: newVel });
    }
  };

  private onPointerUp = (_ev: PointerEvent): void => {
    this.dragTarget = { kind: 'none' };
    this.dragPlane = null;
    // Restore OrbitControls to whatever the current tool requires.
    this.setTool(this.tool);
  };

  private onHover = (_ev: MouseEvent): void => {
    // Hover detection so the cursor tells the user what they would
    // pick up if they click. Cursor updates here are pointer-aware;
    // they do not affect the underlying drag state.
    this.toPointer(_ev);
    const wasArrow = this.hoveringArrow;
    const wasWp    = this.hoveringWp;
    let newArrow = false;
    let newWp    = false;
    if (this.tool === 'vel') {
      const a = this.intersectArrow();
      newArrow = !!a;
    } else if (this.tool === 'move') {
      const w = this.intersectWp();
      newWp = !!w;
    } else if (this.tool === 'add') {
      newWp = false;  // cursor remains the default crosshair
    }
    this.hoveringArrow = newArrow;
    this.hoveringWp    = newWp;
    const canvas = this.canvasRef.nativeElement;
    if (newArrow || newWp) canvas.style.cursor = 'grab';
    else if (wasArrow || wasWp) canvas.style.cursor = '';
  };

  private startDrag(target: DragTarget, ev: PointerEvent): void {
    this.dragTarget = target;
    const wp = this.currentWPs.get((target as any).wpId);
    if (!wp) return;
    const camDir = new THREE.Vector3();
    this.camera.getWorldDirection(camDir);
    this.dragPlane = new THREE.Plane();
    this.dragPlane.setFromNormalAndCoplanarPoint(
      camDir.clone().negate(),
      new THREE.Vector3(wp.pos[0], wp.pos[1], wp.pos[2]),
    );
    this.controls.enabled = false;
    this.canvasRef.nativeElement.setPointerCapture?.(ev.pointerId);
  }

  private findWpMesh(wpId: string): THREE.Mesh | null {
    for (const m of this.wpMeshes.values()) {
      const v = m.get(wpId);
      if (v) return v;
    }
    return null;
  }

  private updateLabelPos(wpId: string, x: number, y: number, z: number): void {
    const mesh = this.findWpMesh(wpId);
    if (!mesh) return;
    const lbl = mesh.userData['label'] as CSS2DObject | undefined;
    if (lbl) lbl.position.set(x, y, z + 3);
  }

  /** Create a new waypoint at the cursor. The other two axes are
   *  fixed by either:
   *    a) the matching `defaultX/Y/Z` value when the camera is in
   *       an axis view that aligns with one of those axes;
   *    b) a raycast against a horizontal plane (Z = defaultZ) when
   *       in the Iso view (free 3D placement).
   */
  private startAddAtCursor(): void {
    const preset = PRESETS.find((p) => p.id === this.activeView);
    if (!preset) return;

    let plane: THREE.Plane;

    if (preset.axis === null) {
      // Iso view — place the waypoint at the cursor's exact 3D
      // position, projected onto a plane perpendicular to the
      // camera (i.e. the screen plane) passing through the
      // world origin. The cursor drives all three coordinates
      // without any default lock-in.
      const camDir = new THREE.Vector3();
      this.camera.getWorldDirection(camDir);
      plane = new THREE.Plane();
      plane.setFromNormalAndCoplanarPoint(
        camDir.negate(),
        new THREE.Vector3(0, 0, 0),
      );
    } else {
      // Axis view — the plane is perpendicular to the camera
      // direction (which is parallel to `preset.axis`). The user
      // controls X/Y freely; the matching axis is locked to its
      // `defaultX/Y/Z` value.
      const planeNormal = new THREE.Vector3(...this.axisVec(preset.axis));
      const planeConst  = -(preset.axis === 'x' ? this.defaultX
                   : preset.axis === 'y' ? this.defaultY
                                         : this.defaultZ);
      plane = new THREE.Plane(planeNormal, planeConst);
    }

    this.raycaster.setFromCamera(this.pointer, this.camera);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(plane, hit)) return;

    const x = +hit.x.toFixed(2);
    const y = +hit.y.toFixed(2);
    const z = +hit.z.toFixed(2);
    this.zone.run(() => this.addWpRequested.emit({ x, y, z }));
  }

  private axisVec(axis: Axis): [number, number, number] {
    if (axis === 'x') return [1, 0, 0];
    if (axis === 'y') return [0, 1, 0];
    return [0, 0, 1];
  }

  private onKeyDown = (ev: KeyboardEvent): void => {
    if (ev.target instanceof HTMLInputElement) return;
    switch (ev.key.toLowerCase()) {
      case 'a': this.setTool('add');  break;
      case 'm': this.setTool('move'); break;
      case 'v': this.setTool('vel');  break;
    }
  };
}

function escapeHtml(s: string): string {
  return s
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}
