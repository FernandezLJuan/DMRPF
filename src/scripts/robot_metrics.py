import os
import math
import re
import matplotlib.pyplot as plt

def find_files(ruta, nombre_mapa):
    archivos_resultado = []
    for archivo in os.listdir(ruta):
        if nombre_mapa in archivo:
            archivos_resultado.append(archivo)
    return archivos_resultado

def parse_file(file_path):
    #store robot related data
    robot_data = []

    #open file
    with open(file_path, 'r') as f:
        lineas = f.readlines()

    #determines wether robot data or global data is being read
    seccion_actual = None
    for linea in lineas:
        linea = linea.strip()
        if not linea or linea.startswith("#"):  #skip empty lines
            if linea.startswith("#Robot Data"):
                seccion_actual = "robot"
            elif linea.startswith("#Global Data"):
                seccion_actual = "global"
            continue

        #read and store robot data
        if seccion_actual == "robot":
            part = linea.split()
            id_robot = int(part[0]) #id of the robot
            tiempo_total = part[1] #can be "null" if the robot has not reached it's goal
            p_size = float(part[2]) #p_size of the robot
            n_requests = int(part[3]) #number of requests

            #don't take metric into accounf if the robot hasn't reached it's goal
            tiempo_total = None if tiempo_total == "null" else float(tiempo_total)
            p_size = None if tiempo_total == "null" else float(p_size)
            n_requests = None if tiempo_total == "null" else float(n_requests)

            #store robot data
            robot_data.append({
                "id_robot": id_robot,
                "tiempo_total": tiempo_total,
                "tamaño_relativo": p_size,
                "n_requests": n_requests,
            })

    return robot_data

#compute metrics grouping them by fleet size
def calcular_metricas_agrupadas(grouped_robot_data, metrica):
    x_values = []
    mean_metrics = []
    std_metrics = []

    for n_robots, datos in grouped_robot_data.items():
        #get metric values for valid robots
        valores_validos = [robot[metrica] for robot in datos if robot[metrica] is not None]

        #compute mean and standard deviation for the metric
        if valores_validos:
            promedio_metrica = sum(valores_validos) / len(valores_validos)
            varianza_metrica = sum((valor - promedio_metrica) ** 2 for valor in valores_validos) / len(valores_validos)
            metric_std = math.sqrt(varianza_metrica)
        else:
            promedio_metrica, metric_std = 0, 0

        #save the number of robots and the metric mean and std
        x_values.append(n_robots)
        mean_metrics.append(promedio_metrica)
        std_metrics.append(metric_std)

    return x_values, mean_metrics, std_metrics

#print mean and std of metrics grouped by number of robots and map
def imprimir_metricas_promedio(nombre_mapa, grouped_robot_data, metrica):
    print(f"\nPromedios y desviación estándar de la métrica '{metrica}' para el mapa '{nombre_mapa}':")
    for n_robots, datos in sorted(grouped_robot_data.items()):
        valores_validos = [robot[metrica] for robot in datos if robot[metrica] is not None]
        if valores_validos:
            promedio_metrica = sum(valores_validos) / len(valores_validos)
            varianza_metrica = sum((valor - promedio_metrica) ** 2 for valor in valores_validos) / len(valores_validos)
            metric_std = math.sqrt(varianza_metrica)
        else:
            promedio_metrica, metric_std = 0, 0
        print(f"Robots: {n_robots}, average: {promedio_metrica}, std_dev: {metric_std}\n")

#plot metrics grouped by number of robots and maps
def graficar_mapas_agrupados(nombre_grupo, mapas, metricas, nombre_metrica):
    plt.figure(figsize=(10, 6))
    #iterate through each map and its associated metrics
    for nombre_mapa, mapa_metricas in metricas.items():

        #extract values to be plotted
        x_values, mean_metrics, std_metrics = mapa_metricas

        #plot metric with standard deviation as error bar
        plt.errorbar(
            x_values, mean_metrics, yerr=std_metrics, fmt='o', capsize=5, label=nombre_mapa
        )

    plt.xlabel("Número de Robots")
    plt.ylabel(f"Promedio de {nombre_metrica.replace('_', ' ').title()}")
    plt.title(f"Promedio de {nombre_metrica.replace('_', ' ').title()} para {nombre_grupo}")
    plt.grid(True)
    plt.legend()
    plt.show()

results_path = "../../results/"
mapas_agrupados = {
    "mapas_reales": {
        'Paris_1_256': (126, 250),
        'warehouse-10-20-10-2-2': (151, 275),
        'warehouse-20-40-10-2-1': (126,250)
    },
    "mapas_conflictivos": {
        'AR0015SR': (1, 204),
        'den520d': (126,250)
    }
}

#select metric to plot
print("\nSelecciona la métrica a representar:")
print("1. tiempo total (tiempo_total)")
print("2. tamaño relativo (tamaño_relativo)")
print("3. número de peticiones (n_requests)")
opcion_metrica = int(input("Ingresa el número correspondiente a la métrica: "))

mapas_metricas = {1: "tiempo_total", 2: "tamaño_relativo", 3: "n_requests"}
nombre_metrica = mapas_metricas.get(opcion_metrica, "tiempo_total")

#loop through each map and its associated range of simulation indices
for nombre_grupo, lista_mapas in mapas_agrupados.items():
    metricas_agrupadas = {}

    #build the path to the folder containing the results for the current map
    for nombre_mapa, (min_range, max_range) in lista_mapas.items():
        ruta_mapa = results_path+nombre_mapa+'/repeated_sims/'

        #find all the result files matching the map name in the folder
        archivos_resultado = find_files(ruta_mapa, nombre_mapa)

        grouped_robot_data = {}

        #process each result file
        for resultados in archivos_resultado:

            #3xtract the simulation index from the filename using a regular expression
            coincidencia = re.search(r'_(\d+)\.dat$', resultados)
            if not coincidencia:
                continue

            num_sim = int(coincidencia.group(1))
            if num_sim < min_range or num_sim > max_range:
                continue

            #build the full file path and parse its data
            file_path = os.path.join(ruta_mapa, resultados)
            robot_data = parse_file(file_path)

            #group robot data by number of robots
            num_robots = len(set(robot["id_robot"] for robot in robot_data))
            if num_robots not in grouped_robot_data:
                #initialize the list for a new number of robots
                grouped_robot_data[num_robots] = []
            
            #add the parsed robot data to the corresponding group
            grouped_robot_data[num_robots].extend(robot_data)

        #compute grouped metrics for current map
        x_values, mean_metrics, std_metrics = calcular_metricas_agrupadas(grouped_robot_data, nombre_metrica)
        metricas_agrupadas[nombre_mapa] = (x_values, mean_metrics, std_metrics)

        #print average and standard deviation for metrics of current map
        imprimir_metricas_promedio(nombre_mapa, grouped_robot_data, nombre_metrica)

    # Graficar mapas agrupados
    graficar_mapas_agrupados(nombre_grupo, lista_mapas, metricas_agrupadas, nombre_metrica)
