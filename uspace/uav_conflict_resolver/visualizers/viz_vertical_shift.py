#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def f(x):
    return np.exp(-x**2)

def main():
    x = np.linspace(-5, 5, 1000)
    k = 0.6
    y = f(x)
    y_shift = f(x) + k

    plt.style.use('classic')
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.plot(x, y, label=r'$f(x)=e^{-x^2}$', color='C0', linewidth=2)
    ax.plot(x, y_shift, label=rf'$f(x)+k,\ k={k}$', color='C2', linewidth=2, linestyle='--')

    # Annotation showing the vertical displacement at x=0
    x0 = 0.0
    ax.annotate('', xy=(x0, f(x0)), xytext=(x0, f(x0) + k), arrowprops=dict(arrowstyle='<->', lw=1.5, color='gray'))
    ax.text(x0 + 0.3, f(x0) + k/2, r'Vertical shift $k$', va='center', fontsize=12, color='gray')

    ax.set_xlabel('x', fontsize=12)
    ax.set_ylabel('y', fontsize=12)
    ax.set_title('Desplazamiento vertical (conceptual)', fontsize=14)
    ax.legend(frameon=False, fontsize=11)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-0.2, 1.8)
    fig.tight_layout()

    out = 'visualizers/vertical_shift.png'
    fig.savefig(out, dpi=300)
    print(f'Saved {out}')

if __name__ == '__main__':
    main()
