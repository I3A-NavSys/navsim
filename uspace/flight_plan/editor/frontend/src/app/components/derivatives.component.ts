import {
  AfterViewInit,
  ChangeDetectionStrategy,
  Component,
  Input,
  OnChanges,
  OnDestroy,
  SimpleChanges,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';


import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  TimeScale,
  Tooltip,
  Legend,
  Filler,
  Title,
} from 'chart.js';
import annotationPlugin from 'chartjs-plugin-annotation';

import {
  Axis,
  AXIS_LABELS,
  DERIVATIVE_LABELS,
  DERIVATIVE_UNITS,
  Derivative,
  FlightPlan,
  Vec3,
  pickAxis,
} from '../services/flight-plan.service';
import { PlanColorService } from '../services/plan-color.service';

Chart.register(
  LineController, LineElement, PointElement,
  LinearScale, TimeScale, Tooltip, Legend, Filler, Title,
  annotationPlugin,
);

/** One Chart.js chart + waypoint marker plugin. We keep one chart per
 *  axis (X, Y, Z) and render every selected derivative for that axis
 *  on the same canvas so multiple derivatives superpose. */
@Component({
  selector: 'app-derivatives',
  standalone: true,
  imports: [CommonModule, FormsModule],
  changeDetection: ChangeDetectionStrategy.OnPush,
  template: `
    <div class="panel-root">
      <header class="panel-header">
        <div class="picker">
          <span class="label">Derivatives</span>
          <ng-container *ngFor="let d of allDerivatives">
            <button class="chip-btn"
                    [class.on]="isSelected(d)"
                    (click)="toggleDerivative(d)">
              {{ labels[d] }}
            </button>
          </ng-container>
        </div>
        <div class="picker">
          <span class="label">Plans</span>
          <ng-container *ngFor="let p of plans">
            <button class="chip-btn"
                    [class.on]="selectedPlans.has(p.id)"
                    [style.--c]="colorFor(p.id)"
                    (click)="togglePlan(p.id)">
              <span class="dot" [style.background]="colorFor(p.id)"></span>
              {{ p.id }}
            </button>
          </ng-container>
        </div>
      </header>

      <div class="charts-grid">
        <div class="chart-cell" *ngFor="let axis of axes">
          <header class="chart-title">
            {{ axisLabel[axis] }} <small>{{ unitLabel() }}</small>
          </header>
          <div class="chart-body">
            <canvas [attr.data-axis]="axis" #chartCanvas></canvas>
          </div>
        </div>
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
    .panel-header {
      display: flex;
      flex-direction: column;
      gap: 6px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--border-soft);
    }
    .picker {
      display: flex;
      align-items: center;
      gap: 6px;
      flex-wrap: wrap;
    }
    .picker .label {
      font-size: 10.5px;
      color: var(--fg-2);
      text-transform: uppercase;
      letter-spacing: 0.06em;
      min-width: 78px;
    }
    .chip-btn {
      background: var(--bg-2);
      border: 1px solid var(--border);
      color: var(--fg-1);
      padding: 3px 8px;
      border-radius: 999px;
      cursor: pointer;
      font-size: 11px;
      transition: all var(--t-fast);
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }
    .chip-btn:hover { background: var(--bg-3); color: var(--fg-0); }
    .chip-btn.on {
      background: color-mix(in srgb, var(--c, var(--accent)) 18%, transparent);
      border-color: color-mix(in srgb, var(--c, var(--accent)) 40%, transparent);
      color: color-mix(in srgb, var(--c, var(--accent)) 75%, white);
    }
    .charts-grid {
      flex: 1 1 0;
      min-height: 0;
      height: 100%;
      display: grid;
      grid-template-rows: 1fr 1fr 1fr;
      gap: 6px;
      padding: 6px;
    }
    .chart-cell {
      background: var(--bg-2);
      border: 1px solid var(--border-soft);
      border-radius: 8px;
      display: flex;
      flex-direction: column;
      min-height: 0;
    }
    .chart-title {
      padding: 4px 12px;
      font-weight: 600;
      font-size: 12px;
      color: var(--fg-1);
      border-bottom: 1px solid var(--border-soft);
      display: flex;
      align-items: center;
      justify-content: space-between;
      background: var(--bg-1);
      border-radius: 8px 8px 0 0;
    }
    .chart-title small {
      color: var(--fg-2);
      font-weight: 400;
    }
    .chart-body {
      position: relative;
      flex: 1;
      min-height: 0;
    }
    canvas {
      position: absolute;
      inset: 0;
      width: 100% !important;
      height: 100% !important;
    }
  `],
})
export class DerivativesComponent implements AfterViewInit, OnChanges, OnDestroy {
  constructor(private colors: PlanColorService) {}
  @Input() plans: FlightPlan[] = [];

  readonly allDerivatives: Derivative[] =
    ['velocity', 'acceleration', 'jerk', 'snap', 'crackle'];
  readonly axes: Axis[] = ['x', 'y', 'z'];
  readonly labels = DERIVATIVE_LABELS;
  readonly axisLabel = AXIS_LABELS;

  /** Stable colour per plan id — delegated to the shared service so
   *  the chart line and the chip button always agree. */
  colorFor(planId: string): string {
    return this.colors.colorFor(planId);
  }

  /** Currently-selected derivative subset (defaults to velocity). */
  selectedDerivatives: Set<Derivative> = new Set(['velocity']);
  /** Currently-selected plan ids. */
  selectedPlans: Set<string> = new Set();

  private charts = new Map<Axis, Chart>();
  private resizeObserver?: ResizeObserver;

  ngAfterViewInit(): void {
    // Defer the chart construction by one frame so that the parent
    // flex/grid layout has had a chance to allocate real heights
    // to each `.chart-cell`. Otherwise Chart.js sees 0×0 and the
    // axes stay invisible.
    setTimeout(() => {
      const allCanvases = Array.from(
        document.querySelectorAll<HTMLCanvasElement>('app-derivatives canvas'),
      );
      for (const axis of this.axes) {
        const canvas = allCanvases.find((c) => c.dataset['axis'] === axis);
        if (!canvas) continue;
        this.charts.set(axis, this.makeChart(canvas, axis));
      }
      this.resizeObserver = new ResizeObserver(() => {
        // Resize each chart to its (now real) container size.
        for (const c of this.charts.values()) c.resize();
        this.apply();
      });
      document
        .querySelectorAll<HTMLElement>('app-derivatives .chart-cell')
        .forEach((c) => this.resizeObserver!.observe(c));
      // Also observe the grid itself — when the user splits the
      // panel, that resizes every cell at once.
      const grid = document.querySelector<HTMLElement>('app-derivatives .charts-grid');
      if (grid) this.resizeObserver.observe(grid);
      this.apply();
    }, 0);
  }

  ngOnChanges(changes: SimpleChanges): void {
    if (!this.charts.size) return;
    if (changes['plans']) {
      const newIds = new Set(this.plans.map((p) => p.id));
      // Drop plans that disappeared.
      for (const id of [...this.selectedPlans]) {
        if (!newIds.has(id)) this.selectedPlans.delete(id);
      }
      // Default-select every VISIBLE plan so the chart shows data
      // the first time the editor loads (instead of an empty canvas).
      if (this.plans.length && this.selectedPlans.size === 0) {
        this.selectedPlans = new Set(
          this.plans.filter((p) => p.visible).map((p) => p.id),
        );
      }
    }
    this.apply();
  }

  ngOnDestroy(): void {
    this.resizeObserver?.disconnect();
    for (const c of this.charts.values()) c.destroy();
  }

  // ----------------------------------------------------------------
  // UI helpers
  // ----------------------------------------------------------------
  isSelected(d: Derivative): boolean {
    return this.selectedDerivatives.has(d);
  }

  toggleDerivative(d: Derivative): void {
    if (this.selectedDerivatives.has(d)) this.selectedDerivatives.delete(d);
    else this.selectedDerivatives.add(d);
    this.apply();
  }

  togglePlan(id: string): void {
    if (this.selectedPlans.has(id)) this.selectedPlans.delete(id);
    else this.selectedPlans.add(id);
    this.apply();
  }

  unitLabel(): string {
    // Show only the most "informative" unit (the derivative with
    // highest order selected); we keep it simple and show the first.
    const d = [...this.selectedDerivatives][0] ?? 'velocity';
    return DERIVATIVE_UNITS[d];
  }

  // ----------------------------------------------------------------
  // Chart construction / update
  // ----------------------------------------------------------------
  private makeChart(canvas: HTMLCanvasElement, axis: Axis): Chart {
    return new Chart(canvas.getContext('2d')!, {
      type: 'line',
      data: { labels: [], datasets: [] },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        parsing: false,
        interaction: { mode: 'nearest', intersect: false },
        scales: {
          x: {
            type: 'linear',
            display: true,
            title: { display: true, text: 'time (s)', color: '#828aa1' },
            ticks: {
              color: '#828aa1',
              callback: (v) => `${Number(v).toFixed(1)}s`,
              maxTicksLimit: 8,
            },
            grid: { color: 'rgba(255,255,255,0.10)' },
            border: { color: 'rgba(255,255,255,0.25)' },
          },
          y: {
            display: true,
            title: { display: true, text: 'value', color: '#828aa1' },
            ticks: {
              color: '#828aa1',
              maxTicksLimit: 6,
              callback: (v) => formatTick(v as number),
            },
            grid: { color: 'rgba(255,255,255,0.06)' },
            border: { color: 'rgba(255,255,255,0.25)' },
          },
        },
        plugins: {
          legend: { display: false },
          tooltip: {
            backgroundColor: 'rgba(17,20,28,0.95)',
            borderColor: '#2a3142',
            borderWidth: 1,
            titleColor: '#f1f5ff',
            bodyColor: '#c9cfdc',
            callbacks: {
              title: (items) => {
                const x = items[0]?.parsed?.x;
                return `t = ${x == null ? '?' : Number(x).toFixed(2)} s`;
              },
              label: (item) => {
                const y = item.parsed?.y;
                return `${item.dataset.label}: ${y == null ? '?' : Number(y).toFixed(3)}`;
              },
            },
          },
        },
      },
    });
  }

  private apply(): void {
    if (!this.charts.size) return;
    // Auto-select every visible plan on the very first call so the
    // chart is never empty at startup.
    if (this.selectedPlans.size === 0 && this.plans.length > 0) {
      this.selectedPlans = new Set(
        this.plans.filter((p) => p.visible).map((p) => p.id),
      );
    }
    const plans =
      this.plans.filter((p) => this.selectedPlans.has(p.id) && p.visible);
    const derivs = [...this.selectedDerivatives];
    for (const axis of this.axes) {
      const chart = this.charts.get(axis);
      if (!chart) continue;
      const datasets: any[] = [];
      const annotations: any = {};
      plans.forEach((plan, pIdx) => {
        const color = this.colors.colorFor(plan.id);
        for (const d of derivs) {
          const series = (plan.trace as any)[d] as Vec3[];
          const data: any[] = new Array(plan.trace.t.length);
          for (let i = 0; i < plan.trace.t.length; i++) {
            data[i] = { x: plan.trace.t[i], y: pickAxis(series[i], axis) };
          }
          datasets.push({
            label: `${plan.id} · ${DERIVATIVE_LABELS[d]}`,
            data,
            borderColor: color,
            backgroundColor: color,
            borderWidth: d === 'velocity' ? 2 : 1.4,
            borderDash: d === 'velocity' ? [] : d === 'acceleration' ? [4, 4] : d === 'jerk' ? [2, 4] : [1, 3],
            pointRadius: 0,
            tension: 0.18,
          });
        }
        // Waypoint markers as vertical annotation lines.
        for (const wp of plan.waypoints) {
          annotations['wp_' + plan.id + '_' + wp.id] = {
            type: 'line',
            xMin: wp.time,
            xMax: wp.time,
            borderColor: hexWithAlpha(color, 0.35),
            borderDash: [3, 3],
            borderWidth: 1,
            label: {
              display: true,
              content: wp.id,
              position: 'start',
              color: hexWithAlpha(color, 0.95),
              backgroundColor: 'rgba(11,13,18,0.85)',
              font: { size: 9, family: 'JetBrains Mono' },
              padding: 2,
            },
          };
        }
      });

      chart.data.datasets = datasets;
      // Re-create options object so the annotations plugin re-evaluates
      // (the plugin caches annotations across updates).
      chart.options.plugins = chart.options.plugins || { legend: {}, tooltip: {} };
      chart.options.plugins.annotation = { annotations };
      chart.update('none');
      chart.resize();   // make sure the canvas uses its real size
    }
  }
}

function hexWithAlpha(hex: string, alpha: number): string {
  // Hex `#rrggbb` → rgba string.
  const m = /^#?([0-9a-f]{6})$/i.exec(hex);
  if (!m) return hex;
  const n = parseInt(m[1], 16);
  const r = (n >> 16) & 0xff;
  const g = (n >> 8)  & 0xff;
  const b =  n        & 0xff;
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function formatTick(v: number): string {
  if (!Number.isFinite(v)) return '';
  if (Math.abs(v) >= 1000)   return v.toExponential(2);
  if (Math.abs(v) >= 100)    return v.toFixed(1);
  if (Math.abs(v) >= 10)     return v.toFixed(2);
  return v.toFixed(3);
}
