# 01: Running your first simulation

En este tutorial aprenderás a ejecutar una simulación abriendo un escenario, 
colocaremos varios drones sobre el terreno y los controlaremos mediante la emisión de comandos básicos.

## Launch Isaac Sim

En primer lugar, abrimos el entorno de simulación NVIDIA Isaac Sim, que ofrecerá un aspecto similar al de la figura siguiente 
(los paneles pueden variar en función de la configuración personal):

![Isaac Sim](./img/isaac_sim.png)

## Launch the scenario


En la ventana de contenidos, buscamos el archivo `ov/assets/worlds/generated_city.usda`, y hacemos doble click sobre él. 
Se abre un escenario de 500x500 metros, con 61 bloques que representan edificios.

![generated_city.usda](./img/generated_city.png)


Con la rueda del ratón hacemos zoom hasta mostrar la azotea de un edificio.
Buscamos en el panel *Content* el archivo `navsim/ov/fleet/UAM_minidrone/UAM_minidrones.usd` y lo arrastramos al escenario 3 veces.
Aparecen tres quadricopteros en escena, elévalos ligeramente por encima del tejado del edificio y pulsa *PLAY*. Los drones caen sobre la superficie.
En el panel *Stage* observamos los tres drones convenientemente renombrados. Podemos estudiar sus componentes internas.


![Minidrones](./img/minidrones.png)
