"""Implementación de Python 3 del algoritmo de Djikstra para encontrar el más corto
camino entre los nodos en un gráfico. Escrito como un ejercicio de aprendizaje, así que muchas
comentarios y sin manejo de errores.
"""
from collections import deque

INFINITY = float("inf")


class Graph:
    def __init__(self, filename):
        """Lee la definición del gráfico y la almacena. Cada línea del gráfico
        archivo de definición define un borde especificando el nodo de inicio,
        nodo final, y distancia, delimitada por espacios.
          Almacena la definición del gráfico en dos propiedades que son utilizadas por
        Algoritmo de Dijkstra en el método shortest_path:
        self.nodes = conjunto de todos los nodos únicos en el gráfico
        self.adjacency_list = dictado que asigna cada nodo a un conjunto desordenado de
        (vecino, distancia) tuplas.


        """

        graph_edges = []
        with open(filename) as fhandle:
            for line in fhandle:
                edge_from, edge_to, cost, *_ = line.strip().split(" ")
                graph_edges.append((edge_from, edge_to, float(cost)))

        self.nodes = set()
        for edge in graph_edges:
            self.nodes.update([edge[0], edge[1]])

        self.adjacency_list = {node: set() for node in self.nodes}
        for edge in graph_edges:
            self.adjacency_list[edge[0]].add((edge[1], edge[2]))

    def shortest_path(self, start_node, end_node):


        unvisited_nodes = self.nodes.copy()  #Nodos Sin Vista.

        # Crea un Diccionario y actualiza la distancia en los Nodos

        distance_from_start = {
            node: (0 if node == start_node else INFINITY) for node in self.nodes
        }

        # Mapea todos los diccionarios
        previous_node = {node: None for node in self.nodes}

        while unvisited_nodes:
            #Nodo no visitado
            current_node = min(
                unvisited_nodes, key=lambda node: distance_from_start[node]
            )
            unvisited_nodes.remove(current_node)

            #Si no estan conectados termina el bucle
            if distance_from_start[current_node] == INFINITY:
                break

            # Para cada vecino de current_node, verifique si la distancia total
            # al vecino a través de current_node es más corto que la distancia que
            # tiene actualmente para ese nodo. Si es así, actualice los valores del vecino
            # para distancia_desde_inicio y nodo_anterior.
            for neighbor, distance in self.adjacency_list[current_node]:
                new_path = distance_from_start[current_node] + distance
                if new_path < distance_from_start[neighbor]:
                    distance_from_start[neighbor] = new_path
                    previous_node[neighbor] = current_node

            if current_node == end_node:
                break # Listo terminamos de visitar nuestro nodo vecino Termina el Bucle

        # Para construir la ruta a ser devuelta, iteramos a través de los nodos desde
        # end_node de vuelta a start_node. Tenga en cuenta el uso de un deque, que puede
        # appendleft con rendimiento O(1).
        path = deque()
        current_node = end_node
        while previous_node[current_node] is not None:
            path.appendleft(current_node)
            current_node = previous_node[current_node]
        path.appendleft(start_node)

        return path, distance_from_start[end_node]


def main():
    """
    ejemplo en caso que requieras Tomar una lista de Ciudades
       """
    verify_algorithm(
        filename="Grafico.txt",
        start="A",
        end="G",
        path=["A", "D", "E", "G"],
        distance=11,
    )
"""    verify_algorithm(
        filename="area.txt",
        start="Bogota",
        end="Manizales",
        path=["Tunja", "Popayan", "Quibdo", "Factoria", "Inirida"],
        distance=10,
    )
    verify_algorithm(
        filename="area.txt",
        start="Cartagena",
        end="Tunja",
        path=["Villavicencio", "Cesar", "Casanare", "Quindio"],
        distance=15,
    )
    verify_algorithm(
        filename="area.txt",
        start="Putumayo",
        end="Narino",
        path=["Magdalena", "SanAndres", "Mocoa", "Cauca", "Arauca"],
        distance=21,
    )"""


def verify_algorithm(filename, start, end, path, distance):
    """Función de ayuda para ejecutar pruebas simples e imprimir resultados en la consola

    filename =  archivo de definición de gráficos
    start/end =  ruta a calcular
    path = ruta corta esperada
    distance = distancia esperada del camino
    """
    graph = Graph(filename)
    returned_path, returned_distance = graph.shortest_path(start, end)

    assert list(returned_path) == path
    assert returned_distance == distance

    print('\narchivo de definición de gráfico: {0}'.format(filename))
    print('       inicio/Fin de  nodes: {0} -> {1}'.format(start, end))
    print('       Ruta mas Corta: {0}'.format(path))
    print('       total distancia: {0}'.format(distance))


if __name__ == "__main__":
    main()