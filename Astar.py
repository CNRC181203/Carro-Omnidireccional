import heapq
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import time
import os
# Creación de clase nodo
class Nodo:
    def __init__(self, posicion, padre=None):
        self.posicion = posicion
        self.padre = padre
        self.g = float('inf')  # Inicializar con infinito para los costos
        self.h = float('inf')
        self.f = float('inf')

    def __eq__(self, otro):
        return self.posicion == otro.posicion

    def __lt__(self, otro):
        return self.f < otro.f

def obtener_camino(nodo):
    camino = []
    actual = nodo
    while actual is not None:
        camino.append(actual.posicion)
        actual = actual.padre
    return camino[::-1]

def es_colision(mapa, x, y, tamano_robot):
    ancho, alto = tamano_robot
    
    if x <= 0 or x + ancho > mapa.shape[0] or y <= 0 or y + alto > mapa.shape[1]:
        return True
    for i in range(int(ancho/2)):
        for j in range(int(ancho/2)):
            if mapa[x + i, y + j] != 0 or mapa[x - i, y - j] != 0:
                return True
    return False
def heuristica(posicion, objetivo):
    return np.sqrt((posicion[0]-objetivo[0])**2+(posicion[1]-objetivo[1])**2)

def Astar(mapa, inicio, objetivo, tamano_robot=(200, 200), paso=200):
    nodos_abiertos = []
    nodos_cerrados = set()
    nodos_explorados = []

    nodo_inicio = Nodo(inicio)
    nodo_inicio.g = 0  # El costo inicial es 0
    nodo_inicio.h = heuristica(inicio, objetivo)
    nodo_inicio.f = nodo_inicio.g+nodo_inicio.h
    nodo_objetivo = Nodo(objetivo)

    heapq.heappush(nodos_abiertos, nodo_inicio)
    costos = {inicio: 0}
    padres = {inicio: None}

    while nodos_abiertos:
        nodo_actual = heapq.heappop(nodos_abiertos)
        nodos_cerrados.add(nodo_actual.posicion)

        if nodo_actual == nodo_objetivo:
            return obtener_camino(nodo_actual), nodos_explorados

        (x, y) = nodo_actual.posicion
        vecinos = [
            (x - paso, y),
            (x + paso, y),
            (x, y - paso),
            (x, y + paso)
        ]

        for siguiente in vecinos:
            (x, y) = siguiente
            if not es_colision(mapa, x, y, tamano_robot):
                nuevo_g = nodo_actual.g + 1
                print(nuevo_g)

                if siguiente not in nodos_cerrados:
                    if siguiente not in costos or nuevo_g < costos[siguiente]:
                        costos[siguiente] = nuevo_g
                        h = heuristica(siguiente, objetivo)
                        f = nuevo_g + h

                        padres[siguiente] = nodo_actual

                        nuevo_nodo = Nodo(siguiente, nodo_actual)
                        nuevo_nodo.g = nuevo_g
                        nuevo_nodo.h = h
                        nuevo_nodo.f = f
                        print(f)
                        heapq.heappush(nodos_abiertos, nuevo_nodo)
                        nodos_explorados.append(siguiente)

    return None, nodos_explorados

# Definición del mapa con paredes de 3 mm y puntos de 20 cm
mapa = np.zeros((2800, 2800))

# Paredes horizontales
mapa[1199:1201, 0:800] = 1
mapa[399:401, 400:1200] = 1
mapa[400*3:400*3+3, 400*3:400*4] = 1
mapa[400*3:400*3+3, 400*6:400*7] = 1
mapa[400*2:400*2+3, 400*3:400*6] = 1
mapa[400*1:400*1+3, 400*4:400*6] = 1

mapa[400*4:400*4+3, 400*1:400*5] = 1
mapa[400*5:400*5+3, 400*2:400*7] = 1
mapa[400*6:400*6+3, 400*1:400*3] = 1
mapa[400*6:400*6+3, 400*4:400*6] = 1

# Paredes verticales
mapa[400*1:400*2+1, 400*1:400*1+3] = 1
mapa[400*2:400*3, 400*2:400*2+3] = 1
mapa[400*0:400*3, 400*3:400*3+3] = 1
mapa[400*0:400*1, 400*4:400*4+3] = 1
mapa[400*2:400*4, 400*5:400*5+3] = 1
mapa[400*3:400*4, 400*6:400*6+3] = 1
mapa[400*4:400*6, 400*1:400*1+3] = 1
mapa[400*5:400*6, 400*4:400*4+3] = 1

# inicio = (2600, 400*3+200)
inicio = (2600, 1400)
objetivo = (200, 200 + 400*3)

t1=time.time()
camino, nodos_explorados = Astar(mapa, inicio, objetivo)
t2=time.time()
print(f"El tiempo de procesamiento es: {t2-t1}")
print(f"numero de nodos explorados: {len(nodos_explorados)}")
# Configuración para visualización
fig = plt.figure(figsize=(15, 10))  # Tamaño de la figura
gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1])  # Mapa a la izquierda, mapa con Dijkstra a la derecha

# Subgráfico para el mapa original
ax0 = plt.subplot(gs[0])
ax0.imshow(mapa, cmap='Greys', origin='upper')
ax0.set_title('Mapa Original')

# Subgráfico para el mapa con el camino de Dijkstra
ax1 = plt.subplot(gs[1])
ax1.imshow(mapa, cmap='Greys', origin='upper')

# Visualización del camino en el mapa con Dijkstra
if camino:
    camino_x, camino_y = zip(*camino)
    ax1.plot(camino_y, camino_x, 'bo-', markersize=10, label='Camino')

# Visualización de los nodos explorados
if nodos_explorados:
    exp_x, exp_y = zip(*nodos_explorados)
    ax1.plot(exp_y, exp_x, 'ro', markersize=5, label='Nodos Explorados')

# Impresión de los puntos del camino
instruction=""
if camino:
    print("Puntos del camino:")
    for punto in camino:
        print(f"({punto[1]}, {punto[0]})")
        instruction+=(f"{{{(punto[0]-200)/1000}, {(punto[1]-200)/1000}, 0}}, ")
        if punto == camino[-1]:
            print(instruction)
    
    # === FILTRAR ESQUINAS ===
    esquinas = []
    for i in range(1, len(camino) - 1):
        prev = camino[i - 1]
        curr = camino[i]
        nxt = camino[i + 1]

        dy1 = curr[0] - prev[0]
        dx1 = curr[1] - prev[1]
        dy2 = nxt[0] - curr[0]
        dx2 = nxt[1] - curr[1]

        # Si cambia la dirección (x o y), y no es en línea recta, es esquina
        if (dx1 != dx2) or (dy1 != dy2):
            esquinas.append(curr)

    # Agregar también inicio y fin si quieres
    if camino:
        esquinas.insert(0, camino[0])
        esquinas.append(camino[-1])

    print("\nEsquinas detectadas:")
    for punto in esquinas:
        print(f"({punto[1]}, {punto[0]})")

# Guardar las esquinas en un archivo .h
script_dir = os.path.dirname(os.path.abspath(__file__))  # Obtiene la ruta del directorio del script
output_path = os.path.join(script_dir, "esquinas.h")  # Combina la ruta del script con el nombre del archivo
try:
    with open(output_path, "w") as f:
        f.write("// Lista de coordenadas de esquinas en metros\n")
        f.write("const float esquinas[][2] = {\n")
        for punto in esquinas:
            # Convertir las coordenadas de milímetros a metros (dividir entre 1000)
            x = punto[1] / 1000.0  # Convertir x a metros
            y = punto[0] / 1000.0  # Convertir y a metros
            f.write(f"    {{ {x:.3f}, {y:.3f} }},\n")  # Escribir en formato {x, y}
        f.write("};\n")
    print(f"Lista de esquinas guardada correctamente en '{output_path}'")
except Exception as e:
    print(f"Error al guardar el archivo: {e}")
# Marcamos el inicio y el objetivo
ax1.plot(inicio[1], inicio[0], 'gs', markersize=10, label='Inicio')
ax1.plot(objetivo[1], objetivo[0], 'ms', markersize=10, label='Objetivo')

# Leyenda y etiquetas
ax1.legend()
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Mapa con Astar')

plt.tight_layout()
plt.show()
