import { Injectable } from '@angular/core';
import { PALETTE } from './flight-plan.service';

/**
 * Singleton service that owns the per-plan colour mapping. Using a
 * service (instead of calling `colorForPlan(planId)` everywhere)
 * guarantees the same colour is shown in the tab strip, the 3D
 * viewer, and the derivatives chart — so the user always sees a
 * single, stable hue per plan id.
 *
 * Allocation policy:
 *   1. If the plan already has a slot, return it.
 *   2. If fewer than `PALETTE.length` plans exist, pick the
 *      lowest unused slot so the first N plans each get a unique
 *      hue.
 *   3. If the palette is exhausted, fall back to a deterministic
 *      djb2 hash of the plan id (cyclic). Prevents the
 *      infinite-loop crash that occurred at ≥ PALETTE.length+1 plans.
 */
@Injectable({ providedIn: 'root' })
export class PlanColorService {
  private slots: Map<string, number> = new Map();
  private nextSlot = 0;

  slotFor(planId: string): number {
    const cached = this.slots.get(planId);
    if (cached !== undefined) return cached;

    if (this.slots.size < PALETTE.length) {
      const used = new Set(this.slots.values());
      let slot = this.nextSlot;
      for (let i = 0; i < PALETTE.length; i++) {
        if (!used.has(slot)) break;
        slot = (slot + 1) % PALETTE.length;
      }
      this.slots.set(planId, slot);
      this.nextSlot = (slot + 1) % PALETTE.length;
      return slot;
    }

    // Exhausted: hash-cyclic allocation.
    let h = 5381;
    for (let i = 0; i < planId.length; i++) {
      h = ((h << 5) + h) + planId.charCodeAt(i);
      h |= 0;
    }
    const slot = Math.abs(h) % PALETTE.length;
    this.slots.set(planId, slot);
    return slot;
  }

  colorFor(planId: string): string {
    return PALETTE[this.slotFor(planId)];
  }

  /** Drop the colour assigned to a plan — used when a plan is
   *  deleted so the slot can be recycled by a future plan. */
  forget(planId: string): void {
    this.slots.delete(planId);
  }
}
