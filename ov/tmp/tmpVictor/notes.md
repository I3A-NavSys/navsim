# FLIGHT PLAN notas

## Asunciones

- El FP se define en un periodo de tiempo. Antes y después, el dron se asume que está en reposo en el waypoint inicial y final respectivamente.

- 


## cosas pendientes

Arreglar FP.PositionFigure y FP.VelocityFigure. quitar la parte del dron. Y ponerla en un objeto monitor.

Repasar FP.Navigation. mejorarlo, no hacer una transición directa del código original del dron en C. Reusar métodos propios del FP y WP. Por ejemplo,
- en lugar de generar un nuevo cómputo del Yaw, usar WP.Course()
- en lugar de calcular cuando varía el rumbo, usar WP.AngleWith()

Rehacer la construcción del FP.
- impedir la teletrasportacion, asignado movimiento rectilíneo uniforme automáticamente
- impedir la manipulación directa de las derivadas de la velocidad
- métodos para:
. Asignar/modificar WP(t,pos,vel)