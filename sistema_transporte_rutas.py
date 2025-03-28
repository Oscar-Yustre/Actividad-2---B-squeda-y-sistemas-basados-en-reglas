# -*- coding: utf-8 -*-
"""
Sistema Inteligente de Búsqueda de Rutas en Transporte Masivo v2
Utiliza el algoritmo A* (con heurística h=0, comportándose como Dijkstra)
para encontrar la ruta de menor costo (tiempo) entre dos estaciones,
indicando la ruta específica a tomar en cada tramo.
"""

import heapq # Para implementar la cola de prioridad eficientemente

def heuristic(node, goal):
    """
    Función heurística. Estima el costo desde 'node' hasta 'goal'.
    Usamos h=0 para que A* se comporte como Dijkstra.
    """
    return 0

def reconstruct_detailed_itinerary(parents, g_costs, goal_node, start_node):
    """
    Reconstruye el itinerario detallado (incluyendo rutas y tiempos por tramo)
    desde el nodo inicial hasta el nodo objetivo.
    """
    itinerary = []
    current = goal_node

    # Si el inicio y fin son iguales
    if start_node == goal_node:
        return itinerary # Itinerario vacío, ya estás ahí

    # Retroceder desde el nodo objetivo usando el diccionario 'parents'
    while current in parents and parents[current] is not None:
        parent_node, route_taken = parents[current]

        # Calcular el tiempo de este segmento específico
        # Puede haber pequeñas diferencias por flotantes si los costos no son enteros
        cost_segment = round(g_costs[current] - g_costs[parent_node], 1)

        # Añadir el paso al PRINCIPIO de la lista para que quede en orden
        itinerary.insert(0, {
            'from': parent_node,
            'to': current,
            'route': route_taken,
            'time': cost_segment
        })
        current = parent_node # Moverse al padre para el siguiente paso

    # Verificar si llegamos al nodo de inicio (deberíamos si hay ruta)
    if current != start_node:
        # Esto no debería pasar si a_star encontró una ruta válida
        return None # Indica un error en la reconstrucción

    return itinerary


def a_star_search_with_routes(graph, start_node, goal_node):
    """
    Implementación del algoritmo A* modificado para rastrear rutas.

    Args:
        graph (dict): El grafo. Formato:
                      {'A': {'B': (costo, ruta), 'C': (costo, ruta)}, ...}
        start_node (str): Nodo inicial.
        goal_node (str): Nodo objetivo.

    Returns:
        tuple: (costo_total, parents_dict, g_costs_dict) si se encuentra ruta,
               (float('inf'), {}, {}) si no.
               parents_dict tiene formato {hijo: (padre, ruta_usada)}
    """
    # Cola de prioridad: almacena (f_cost, node)
    open_set = [(heuristic(start_node, goal_node), start_node)] # (f_cost inicial, nodo inicial)

    # Costo real (g_cost) desde el inicio hasta un nodo
    g_costs = {node: float('inf') for node in graph}
    g_costs[start_node] = 0

    # Reconstrucción del camino {nodo_hijo: (nodo_padre, ruta_usada_para_llegar)}
    parents = {start_node: None}

    while open_set:
        current_f_cost, current_node = heapq.heappop(open_set)

        # Optimización: Si ya procesamos este nodo con un costo menor, saltar.
        if current_f_cost > g_costs[current_node] + heuristic(current_node, goal_node):
             continue

        if current_node == goal_node:
            return g_costs[current_node], parents, g_costs # Devolver costo, padres y g_costs

        if current_node in graph:
            for neighbor, edge_data in graph[current_node].items():
                cost, route_name = edge_data # Extraer costo y nombre de ruta
                tentative_g_cost = g_costs[current_node] + cost

                if tentative_g_cost < g_costs.get(neighbor, float('inf')):
                    parents[neighbor] = (current_node, route_name) # Guardar padre Y RUTA
                    g_costs[neighbor] = tentative_g_cost
                    h_cost = heuristic(neighbor, goal_node)
                    f_cost = tentative_g_cost + h_cost
                    heapq.heappush(open_set, (f_cost, neighbor))

    return float('inf'), {}, {} # No se encontró ruta

# --- Definición del Sistema de Transporte (MODIFICADO con Rutas) ---
# Ahora, el valor asociado a cada vecino es una tupla: (tiempo, nombre_ruta)

sistema_transporte_con_rutas = {
    'Portal_Norte': {'Calle_146': (8, 'B12'), 'Alcala': (12, 'G11')},
    'Calle_146': {'Portal_Norte': (8, 'B12'), 'Calle_127': (5, 'B12')},
    'Alcala': {'Portal_Norte': (12, 'G11'), 'Pepe_Sierra': (6, 'G11')},
    'Calle_127': {'Calle_146': (5, 'B12'), 'Pepe_Sierra': (4, 'B74'), 'Calle_100': (7, 'B12')},
    'Pepe_Sierra': {'Alcala': (6, 'G11'), 'Calle_127': (4, 'B74'), 'Calle_100': (5, 'G11')},
    'Calle_100': {'Calle_127': (7, 'B12'), 'Pepe_Sierra': (5, 'G11'), 'Virrey': (6, 'B74'), 'Heroes': (10, 'B12')},
    'Virrey': {'Calle_100': (6, 'B74'), 'Calle_85': (4, 'B74')},
    'Heroes': {'Calle_100': (10, 'B12'), 'Calle_76': (5, 'B12')},
    'Calle_85': {'Virrey': (4, 'B74'), 'Calle_76': (3, 'G11')},
    'Calle_76': {'Heroes': (5, 'B12'), 'Calle_85': (3, 'G11'), 'Polo': (6, 'G11')},
    'Polo': {'Calle_76': (6, 'G11')}
}

# Asegurarse de que todos los nodos existan como claves principales en el grafo
all_nodes_rutas = set(sistema_transporte_con_rutas.keys())
for node, neighbors in sistema_transporte_con_rutas.items():
    all_nodes_rutas.update(neighbors.keys())
for node in all_nodes_rutas:
    if node not in sistema_transporte_con_rutas:
        sistema_transporte_con_rutas[node] = {}


# --- Ejecución Principal Modificada ---
if __name__ == "__main__":
    print("\n--- Sistema de Búsqueda de Rutas v2 (con Detalle de Rutas) ---")
    print("Estaciones disponibles:", sorted(list(all_nodes_rutas)))

    while True:
        try:
            inicio = input("Ingrese la estación de inicio: ")
            if inicio not in all_nodes_rutas:
                print("Error: Estación de inicio no válida.")
                continue

            fin = input("Ingrese la estación de destino: ")
            if fin not in all_nodes_rutas:
                print("Error: Estación de destino no válida.")
                continue

            if inicio == fin:
                print("\nLa estación de inicio y destino son la misma.")
                print("No necesitas tomar ninguna ruta.")
                print("Tiempo total estimado: 0 minutos")
            else:
                costo_total, parents_dict, g_costs_dict = a_star_search_with_routes(sistema_transporte_con_rutas, inicio, fin)

                if costo_total == float('inf'):
                    print(f"\nNo se encontró una ruta entre '{inicio}' y '{fin}'.")
                else:
                    print(f"\nLa mejor ruta encontrada entre '{inicio}' y '{fin}' es:")
                    itinerario = reconstruct_detailed_itinerary(parents_dict, g_costs_dict, fin, inicio)

                    if itinerario is None:
                         print("Error al reconstruir el itinerario.")
                    elif not itinerario:
                         # Esto no debería pasar si costo != inf y inicio != fin
                         print("Itinerario vacío, posible error.")
                    else:
                        for i, paso in enumerate(itinerario):
                            print(f"  Paso {i+1}: Tomar Ruta '{paso['route']}' desde '{paso['from']}' hasta '{paso['to']}' (Tiempo aprox: {paso['time']} min)")
                        print(f"\nTiempo total estimado del viaje: {round(costo_total,1)} minutos")


            continuar = input("\n¿Desea buscar otra ruta? (s/n): ").lower()
            if continuar != 's':
                break
        except EOFError:
             print("\nEntrada no disponible, finalizando.")
             break
        except Exception as e:
             print(f"\nOcurrió un error inesperado: {e}")
             break

    print("\n--- Fin del programa v2 ---")
