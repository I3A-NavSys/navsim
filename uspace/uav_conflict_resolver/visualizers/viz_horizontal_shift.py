#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def f(x):
    return np.exp(-x**2)

def main():
    x = np.linspace(-5, 5, 1000)
    h = 1.5
    y = f(x)
    y_shift = f(x - h)

    plt.style.use('classic')
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.plot(x, y, label=r'$f(x)=e^{-x^2}$', color='C0', linewidth=2)
    ax.plot(x, y_shift, label=rf'$f(x-h),\ h={h}$', color='C1', linewidth=2, linestyle='--')

    # Annotation showing the horizontal displacement between peaks
    ax.annotate('', xy=(0, f(0)), xytext=(h, f(0 - h)), arrowprops=dict(arrowstyle='<->', lw=1.5, color='gray'))
    ax.text(h/2, 0.85, r'Horizontal shift $h$', ha='center', fontsize=12, color='gray')

    # Axes, labels, title
    ax.set_xlabel('x', fontsize=12)
    ax.set_ylabel('y', fontsize=12)
    ax.set_title('Desplazamiento horizontal (conceptual)', fontsize=14)
    ax.legend(frameon=False, fontsize=11)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-0.1, 1.1)
    fig.tight_layout()

    out = 'visualizers/horizontal_shift.png'
    fig.savefig(out, dpi=300)
    print(f'Saved {out}')

if __name__ == '__main__':
    main()
