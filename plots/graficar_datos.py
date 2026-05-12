import matplotlib.pyplot as plt
import pandas as pd

# Cargamos los datos desde el archivo de texto
nombre_archivo = 'resultados.txt'
try:
    df = pd.read_csv(nombre_archivo)
except FileNotFoundError:
    print(f"Error: No se encontró el archivo '{nombre_archivo}'. Asegúrate de que esté en la misma carpeta.")
    exit()

# Configuramos el tamaño y estilo de la figura
plt.figure(figsize=(12, 6))

# Graficamos las tres señales con estilos distintos para mejor comprensión
# - Crudo: Punteado y semi-transparente para que no tape lo importante
plt.plot(df['Tiempo(s)'], df['Crudo(m)'],
         label='Sensor Crudo', color='lightcoral', alpha=0.7, linestyle=':')

# - Filtro EMA: Línea discontinua azul
plt.plot(df['Tiempo(s)'], df['Filtro_EMA(m)'],
         label='Filtro Simple (EMA)', color='royalblue', linestyle='--')

# - Kalman: Línea sólida, más gruesa y verde
plt.plot(df['Tiempo(s)'], df['Kalman(m)'],
         label='Filtro de Kalman', color='forestgreen', linewidth=2)

# Dibujamos una línea roja que marque el umbral de seguridad
UMBRAL_SEGURO = 0.09
plt.axhline(y=UMBRAL_SEGURO, color='red', linestyle='-.', alpha=0.5, label='Umbral de Giro (0.09m)')

# Personalización de etiquetas, títulos y diseño
plt.title('Navegación Reactiva: Fusión Sensorial con Filtro de Kalman', fontsize=14, fontweight='bold')
plt.xlabel('Tiempo de Simulación (segundos)', fontsize=12)
plt.ylabel('Distancia Frontal Estimada (metros)', fontsize=12)

# Añadimos la leyenda y una cuadrícula
plt.legend(loc='lower right', fontsize=10)
plt.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()

plt.savefig('grafico_kalman.png', dpi=300)
print("Gráfico generado y guardado como 'grafico_kalman.png'")

plt.show()
