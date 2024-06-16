#Practica3: SIMULADOR DE ALGORITMO DE DIJKSTRA
#Alumno: Ivan Dominguez
#Registro: 21310234
#Grupo: 6E1

import heapq
import matplotlib.pyplot as plt
import networkx as nx

def dibujar_grafo(grafo, distancias, nodo_actual, visitados, camino):
    G = nx.Graph()
    
    # Añade las aristas al grafo de NetworkX con sus pesos
    for nodo in grafo:
        for vecino, peso in grafo[nodo].items():
            G.add_edge(nodo, vecino, weight=peso)

    pos = nx.spring_layout(G)

    # Dibujar nodos
    colores_nodos = ['green' if nodo in visitados else 'red' for nodo in G.nodes()]
    nx.draw_networkx_nodes(G, pos, node_color=colores_nodos)

    # Dibujar aristas
    nx.draw_networkx_edges(G, pos)
    
    # Dibujar etiquetas de los nodos con sus distancias
    etiquetas_nodos = {nodo: f"{nodo}\n{distancias[nodo]:.1f}" for nodo in G.nodes()}
    nx.draw_networkx_labels(G, pos, labels=etiquetas_nodos)

    # Dibujar etiquetas de las aristas con sus pesos
    etiquetas_aristas = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=etiquetas_aristas)
    
    plt.title(f"Visitando nodo: {nodo_actual}")
    plt.show()

def dijkstra(grafo, inicio):
    # Inicialización
    cola = []
    heapq.heappush(cola, (0, inicio))
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[inicio] = 0
    visitados = set()
    camino = {nodo: None for nodo in grafo}
    
    while cola:
        distancia_actual, nodo_actual = heapq.heappop(cola)
        
        if nodo_actual in visitados:
            continue
        
        visitados.add(nodo_actual)
        
        # Proceso principal
        for vecino, peso in grafo[nodo_actual].items():
            distancia = distancia_actual + peso
            
            # Si se encuentra una distancia más corta
            if distancia < distancias[vecino]:
                distancias[vecino] = distancia
                heapq.heappush(cola, (distancia, vecino))
                camino[vecino] = nodo_actual
        
        # Simulación paso a paso con gráficos
        dibujar_grafo(grafo, distancias, nodo_actual, visitados, camino)
    
    return distancias, camino

# Ejemplo de grafo
grafo = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

# Simulación del algoritmo de Dijkstra
nodo_inicio = 'A'
distancias, camino = dijkstra(grafo, nodo_inicio)

print("Distancias finales:", distancias)
print("Ruta más corta:", camino)


