import { Injectable } from '@angular/core';
import {
  BehaviorSubject,
  Observable,
  Subject,
  of,
} from 'rxjs';
import {
  catchError,
  debounceTime,
  distinctUntilChanged,
  filter,
  finalize,
  map,
  share,
  switchMap,
  tap,
} from 'rxjs/operators';
import { FlightPlan, FlightPlanService, Vec3 } from './flight-plan.service';

/**
 * One user-issued change that maps to exactly one HTTP call.
 * All mutations flow through `mutations$` so the latest request
 * always wins (switchMap) and rapid drags are coalesced (debounce).
 *
 * `_nonce` is auto-injected by `push()` so callers can omit it.
 */
export type Mutation =
  | { kind: 'add';        planId: string; payload: MutationAddPayload; _nonce?: number }
  | { kind: 'patch';      planId: string; payload: MutationPatchPayload; _nonce?: number }
  | { kind: 'delete';     planId: string; id: string; _nonce?: number }
  | { kind: 'connect';    planId: string; _nonce?: number }
  | { kind: 'planAttr';   planId: string; payload: MutationPlanAttrPayload; _nonce?: number }
  | { kind: 'visibility'; planId: string; visible: boolean; _nonce?: number }
  | { kind: 'planCreate'; payload: MutationPlanAttrPayload; _nonce?: number }
  | { kind: 'planDelete'; planId: string; _nonce?: number }
  | { kind: 'simSet';     payload: { reset?: boolean; time?: number }; _nonce?: number };

export interface MutationAddPayload {
  id?: string;
  time: number;
  pos: Vec3;
  vel?: Vec3;
  fly_over?: boolean;
  heading?: [number, number];
}

export interface MutationPatchPayload {
  id: string;
  time?: number;
  pos?: Vec3;
  vel?: Vec3;
  fly_over?: boolean;
  heading?: [number, number];
}

export interface MutationPlanAttrPayload {
  id?: string;
  priority?: number;
  radius?: number;
  max_var_lin_vel?: number;
  max_var_ang_vel?: number;
  waypoints?: any[];
}

/** Stream of every FlightPlan mutation that returns either a
 *  `FlightPlan` (regular mutations) or a sentinel event for plan
 *  lifecycle changes (`planCreate`, `planDelete`). The latest
 *  emission is always the one being applied. */
export type MutationResult = FlightPlan | PlanDeletedEvent;

export interface PlanDeletedEvent {
  /** Discriminator — `__plan_deleted__`. */
  kind: '__plan_deleted__';
  planId: string;
}

export function isPlanDeleted(r: MutationResult): r is PlanDeletedEvent {
  return typeof r === 'object' && r !== null && (r as any).kind === '__plan_deleted__';
}

export interface MutationStream {
  /** Coalesced mutation stream. Last write wins. */
  mutate$: Observable<MutationResult>;
  /** Push a new mutation on the stream. */
  push(m: Mutation): void;
  /** Observable that emits the current in-flight flag. */
  busy$: Observable<boolean>;
  /** Observable that emits the current error message (or null). */
  error$: Observable<string | null>;
}

@Injectable({ providedIn: 'root' })
export class MutationStreamService implements MutationStream {
  private readonly mutations$ = new Subject<Mutation>();
  private readonly _busy$  = new BehaviorSubject<boolean>(false);
  private readonly _error$ = new BehaviorSubject<string | null>(null);
  /** Monotonic nonce injected on every `push()` so identical
   *  mutations are not collapsed by `distinctUntilChanged`. */
  private nonce = 0;

  readonly mutate$: Observable<MutationResult>;
  readonly busy$:   Observable<boolean>;
  readonly error$:  Observable<string | null>;

  constructor(private fp: FlightPlanService) {
    this.busy$  = this._busy$.asObservable();
    this.error$ = this._error$.asObservable();

    this.mutate$ = this.mutations$.pipe(
      filter((m) => m !== null && m !== undefined),
      debounceTime(20),
      distinctUntilChanged((a, b) => JSON.stringify(a) === JSON.stringify(b)),
      tap(() => {
        this._busy$.next(true);
        this._error$.next(null);
      }),
      switchMap((m) => this.dispatch(m)),
      tap(() => this._busy$.next(false)),
      catchError((err) => {
        this._error$.next(this.fmtError(err));
        this._busy$.next(false);
        // Don't kill the stream — keep subsequent mutations working.
        return of(this.emptyPlan('__error__') as MutationResult);
      }),
      finalize(() => this._busy$.next(false)),
      share(),
    );
  }

  push(m: Mutation): void {
    // Inject a fresh nonce if the caller didn't supply one — this
    // guarantees that two consecutive "New plan" clicks are not
    // collapsed into a single HTTP request by `distinctUntilChanged`.
    (m as { _nonce?: number })._nonce = (m as { _nonce?: number })._nonce ?? ++this.nonce;
    this.mutations$.next(m);
  }

  private dispatch(m: Mutation): Observable<MutationResult> {
    switch (m.kind) {
      case 'add':
        return this.fp.addWaypoint(m.planId, m.payload);
      case 'patch':
        return this.fp.updateWaypoint(m.planId, m.payload);
      case 'delete':
        return this.fp.deleteWaypoint(m.planId, m.id);
      case 'connect':
        return this.fp.connect(m.planId);
      case 'planAttr':
        return this.fp.updateAttributes(m.planId, m.payload);
      case 'visibility':
        return this.fp.setVisibility(m.planId, m.visible);
      case 'planCreate':
        return this.fp.create(m.payload);
      case 'planDelete':
        return this.fp.delete(m.planId).pipe(
          map(() => ({ kind: '__plan_deleted__', planId: m.planId } as PlanDeletedEvent)),
        );
      case 'simSet':
        return this.fp.setSimTime(m.payload.time ?? null).pipe(
          // simSet returns a number — wrap into a synthetic plan so the
          // stream type stays uniform. Only used in callers that
          // specifically subscribe to it.
          map(() => this.emptyPlan('sim')),
        );
    }
  }

  private emptyPlan(id: string): FlightPlan {
    return {
      id,
      priority: 0,
      radius: 0,
      max_var_lin_vel: 0,
      max_var_ang_vel: 0,
      waypoints: [],
      length: 0,
      start_time: null,
      finish_time: null,
      trace: {
        dt: 0.05,
        t: [],
        pos: [],
        vel: [],
        acel: [],
        jerk: [],
        snap: [],
        crakle: [],
      },
      visible: true,
    };
  }

  private fmtError(_e: unknown): string {
    return 'request failed';
  }
}
