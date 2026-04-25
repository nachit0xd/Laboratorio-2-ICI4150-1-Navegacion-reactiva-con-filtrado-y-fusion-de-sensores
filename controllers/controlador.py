from controller import Robot

# 1. Creamos la instancia del robot y definimos el tiempo de muestreo
robot = Robot()
TIME_STEP = 50 # Frecuencia de muestreo (Ts = 0.05 s)

# VARIABLES PARA EL LÍMITE DE SIMULACIÓN
step_count = 0

# 30 segundos de simulación (30s / 0.05s por paso = 600 pasos)
MAX_STEPS = 600

# INICIALIZACIÓN DE MOTORES
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# INICIALIZACIÓN DE ENCODERS
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# INICIALIZACIÓN DE SENSORES DE DISTANCIA
# En el caso del e-puck: ps0 y ps7 son los frontales, ps2 es derecho, ps5 es izquierdo
front_right_sensor = robot.getDevice('ps0')
front_left_sensor = robot.getDevice('ps7')
right_sensor = robot.getDevice('ps2')
left_sensor = robot.getDevice('ps5')

# Habilitamos los sensores con el mismo tiempo de muestreo
front_right_sensor.enable(TIME_STEP)
front_left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)
left_sensor.enable(TIME_STEP)

# PARÁMETROS DEL ROBOT
WHEEL_RADIUS = 0.0205 
MAX_SPEED = 6.28 # Velocidad máxima típica del e-puck, en rad/s

# VARIABLES DEL FILTRO DE KALMAN
# Valores iniciales
d_est = 0.06 # Distancia inicial estimada (ej: 0.06 metros libre al frente)
P = 1.0     # Incertidumbre inicial (covarianza)

# Constantes de sintonización del filtro (se pueden ajustar para mejorar el rendimiento)
Q = 0.05 # Ruido del proceso (incertidumbre en nuestra odometría/motores)
R = 0.01  # Ruido de la medición (varianza/incertidumbre del sensor infrarrojo)

# VARIABLES DE ALMACENAMIENTO
raw_front_data = [] # Para guardar las lecturas crudas convertidas a metros (z_k) para gráficos finales
filtered_front_data = [] # Para guardar las lecturas filtradas por EMA para gráficos finales
kalman_front_data = [] # Para guardar la estimación fusionada

# Variables auxiliares
prev_left_enc = 0.0
prev_right_enc = 0.0
first_step = True
alpha = 0.2
filtered_val = 0.0

# Umbral de seguridad para la navegación (en metros)
SAFE_DIST = 0.04 # Si el Kalman estima que hay un obstáculo a 4 cm o menos, se activa la evasión forzada

# Un temporizador para obligar al robot a completar el giro, importante para evitar que se quede atascado en situaciones de obstáculos cercanos
turn_timer = 0

# Función que convierte la lectura cruda del sensor (0-4095) a una distancia en metros, basada en una tabla de conversión o fórmula específica del sensor
def convertir_sensor_a_metros(valor_crudo):
    distancia_maxima = 0.06 
    
    if valor_crudo <= 10: 
        return distancia_maxima # Camino libre
    elif valor_crudo >= 4000:
        return 0.005 # Chocando inminentemente
    
    return distancia_maxima - (valor_crudo / 4095.0) * distancia_maxima

# INICIO DEL CONTROLADOR
print("Controlador iniciado. Comenzando simulación...")

while robot.step(TIME_STEP) != -1:
    
    # 1. LECTURA Y CONVERSIÓN DE SENSORES
    val_fr = front_right_sensor.getValue()
    val_fl = front_left_sensor.getValue()
    # Tomamos el valor más alto de los dos sensores frontales
    raw_front_adc = max(val_fr, val_fl)
    
    # Transformamos la lectura a metros para que coincida con la odometría
    z_k = convertir_sensor_a_metros(raw_front_adc) 

    left_enc_val = left_encoder.getValue()
    right_enc_val = right_encoder.getValue()

    if first_step:
        prev_left_enc = left_enc_val
        prev_right_enc = right_enc_val
        first_step = False

    # 2. CÁLCULO DE LA ODOMETRÍA (Avance del robot basado en encoders)
    delta_theta_left = left_enc_val - prev_left_enc
    delta_theta_right = right_enc_val - prev_right_enc
    delta_d = (WHEEL_RADIUS * delta_theta_left + WHEEL_RADIUS * delta_theta_right) / 2.0
    
    prev_left_enc = left_enc_val
    prev_right_enc = right_enc_val

    # 3. FILTRO SIMPLE (EMA) PARA SUAVIZAR LA LECTURA CRUDA
    if len(raw_front_data) == 0:
        filtered_val = z_k 
    else:
        filtered_val = (alpha * z_k) + ((1.0 - alpha) * filtered_val)

    # 4. FILTRO DE KALMAN (FUSIÓN SENSORIAL)
    
    # ETAPA DE PREDICCIÓN
    # El laboratorio define la predicción como la suma del estado anterior y el avance
    d_pred = d_est - delta_d 
    
    # Proyectamos la incertidumbre sumando el ruido del proceso (Q)
    P_pred = P + Q 
    
    # ETAPA DE ACTUALIZACIÓN
    # Calculamos la Ganancia de Kalman
    K = P_pred / (P_pred + R) 
    
    # Actualizamos la estimación fusionando la predicción y la medición 
    d_est = d_pred + K * (z_k - d_pred) 
    
    # Actualizamos la incertidumbre para la próxima iteración
    P = (1.0 - K) * P_pred 

    # 5. NAVEGACIÓN REACTIVA (Con evasión forzada)
    
    val_left = left_sensor.getValue()
    val_right = right_sensor.getValue()
    
    vL = 0.0
    vR = 0.0

    # Si detectamos un obstáculo, activamos el temporizador de giro (turn_timer) para forzar al robot a completar un giro evasivo, incluso si el sensor frontal ya no detecta el obstáculo
    if d_est <= SAFE_DIST:
        turn_timer = 10 

    # LÓGICA DE MOVIMIENTO
    if turn_timer > 0:
        # MIENTRAS EL TEMPORIZADOR ESTÉ ACTIVO: GIRO EVASIVO, no importa lo que diga el sensor frontal. Esto ayuda a evitar que el robot se quede atascado en situaciones de obstáculos cercanos
        if val_left > val_right:
            vL = MAX_SPEED 
            vR = -MAX_SPEED 
        else:
            vL = -MAX_SPEED 
            vR = MAX_SPEED 
            
        turn_timer -= 1 # Restamos 1 al temporizador en cada ciclo
        
    else:
        # CAMINO LIBRE Y TEMPORIZADOR EN CERO, AVANZAMOS CON VELOCIDAD MODERADA
        vL = MAX_SPEED * 0.4
        vR = MAX_SPEED * 0.4

    # Imprimir para depurar y ver cómo evolucionan las lecturas y la acción tomada
    print(f"Crudo(m): {z_k:.3f} | Kalman(m): {d_est:.3f} | Acción: {'AVANZA' if d_est > SAFE_DIST else 'GIRA'}")

    # Aplicamos velocidades
    left_motor.setVelocity(vL)
    right_motor.setVelocity(vR)

    # 6. ALMACENAMIENTO DE DATOS PARA GRÁFICOS FINALES
    raw_front_data.append(z_k)
    filtered_front_data.append(filtered_val)
    kalman_front_data.append(d_est)

    # 7. FINALIZACIÓN Y EXTRACCIÓN DE DATOS
    step_count += 1
    
    if step_count >= MAX_STEPS:
        # Detenemos los motores
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        print("\n" + "="*50)
        print(" SIMULACIÓN TERMINADA. COPIA LOS DATOS DE ABAJO")
        print("="*50)
        
        # Imprimimos el encabezado
        print("Tiempo(s),Crudo(m),Filtro_EMA(m),Kalman(m)")
        
        # Recorremos los arreglos e imprimimos fila por fila
        for i in range(len(raw_front_data)):
            # Calculamos el tiempo transcurrido en segundos para esta muestra
            tiempo_seg = (i * TIME_STEP) / 1000.0 
            
            print(f"{tiempo_seg:.2f},{raw_front_data[i]:.4f},{filtered_front_data[i]:.4f},{kalman_front_data[i]:.4f}")
            
        print("="*50)
        break # Rompe el bucle while y detiene el controlador por completo
