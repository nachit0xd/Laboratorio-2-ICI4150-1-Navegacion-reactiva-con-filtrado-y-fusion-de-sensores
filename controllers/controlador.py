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
Q = 0.01 # Ruido del proceso (incertidumbre en nuestra odometría/motores)
R = 0.05  # Ruido de la medición (varianza/incertidumbre del sensor infrarrojo)

# VARIABLES DE ALMACENAMIENTO
raw_front_data = [] # Para guardar las lecturas crudas convertidas a metros (z_k) para gráficos finales
filtered_front_data = [] # Para guardar las lecturas filtradas por EMA para gráficos finales
kalman_front_data = [] # Para guardar la estimación fusionada

# Variables auxiliares
prev_left_enc = 0.0
prev_right_enc = 0.0
first_step = True
alpha = 0.3
filtered_val = 0.0

# Umbral de seguridad para la navegación (en metros)
SAFE_DIST = 0.09 # Si el Kalman estima que hay un obstáculo a 9 cm o menos, se activa la evasión forzada

# Un temporizador para obligar al robot a completar el giro, importante para evitar que se quede atascado en situaciones de obstáculos cercanos
turn_timer = 0

# Variables para modo de evasión
evasion_mode = False 
evasion_steps = 0
MAX_EVASION_STEPS = 45  # ~2.25 segundos de evasión

# Función que convierte la lectura cruda del sensor (0-4095) a una distancia en metros, basada en una tabla de conversión o fórmula específica del sensor
def convertir_sensor_a_metros(valor_crudo):
    if valor_crudo <= 50:
        return 0.10  # Libre (>10cm)
    elif valor_crudo >= 3800:
        return 0.01  # Muy cerca (1cm)
    
    # Modelo ajustado al rango del e-puck
    distancia = 0.12 * (1 - valor_crudo / 4000.0)
    return max(0.01, min(0.12, distancia))

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

    # Evitamos que la predicción sea negativa
    if d_pred < 0.01:
        d_pred = 0.01
    
    # Proyectamos la incertidumbre sumando el ruido del proceso (Q)
    P_pred = P + Q 
    
    # ETAPA DE ACTUALIZACIÓN
    # Calculamos la Ganancia de Kalman
    K = P_pred / (P_pred + R) 
    
    # Actualizamos la estimación fusionando la predicción y la medición 
    d_est = d_pred + K * (z_k - d_pred) 

    # Ajustamos la estimación a un rango realista
    d_est = max(0.01, min(0.12, d_est))
    
    # Actualizamos la incertidumbre para la próxima iteración
    P = (1.0 - K) * P_pred 

    # 5. NAVEGACIÓN REACTIVA (Con evasión forzada)
    val_left = left_sensor.getValue()
    val_right = right_sensor.getValue()
    val_fr = front_right_sensor.getValue()
    val_fl = front_left_sensor.getValue()

    # Detección de peligro
    avg_front = (val_fr + val_fl) / 2.0
    front_dist = convertir_sensor_a_metros(avg_front)
    left_dist = convertir_sensor_a_metros(val_left) if val_left > 50 else 0.12
    right_dist = convertir_sensor_a_metros(val_right) if val_right > 50 else 0.12
    
    vL = 0.0
    vR = 0.0

    # LÓGICA DE EVASIÓN y MOVIMIENTO
    # Si el Kalman estima que hay un obstáculo a 9 cm o menos, o si el sensor frontal detecta algo muy cercano, activamos la evasión forzada
    if d_est <= SAFE_DIST or front_dist <= SAFE_DIST:
        if evasion_steps == 0:
            evasion_mode = True
            evasion_steps = MAX_EVASION_STEPS
            print(f"OBSTÁCULO DETECTADO! d_est={d_est:.3f}m, front_dist={front_dist:.3f}m") # Agregamos esta impresión para ver cuándo se activa la evasión

    # Si estamos en modo de evasión, seguimos la estrategia definida, que incluye una fase de retroceso antes del giro, y luego una fase de reanudación gradual
    if evasion_steps > 0:
        # ESTRATEGIA DE EVASIÓN: Retroceder y girar
        if evasion_steps > MAX_EVASION_STEPS * 0.7:
            # FASE 1: Retroceder (0.5-1 segundo)
            vL = -MAX_SPEED * 0.6
            vR = -MAX_SPEED * 0.6
        elif evasion_steps > MAX_EVASION_STEPS * 0.3:
            # FASE 2: Girar hacia el lado con más espacio
            if left_dist > right_dist:
                vL = -MAX_SPEED * 0.5
                vR = MAX_SPEED * 0.5
                print("  Giro a la izquierda")
            else:
                vL = MAX_SPEED * 0.5
                vR = -MAX_SPEED * 0.5
                print("  Giro a la derecha")
        else:
            # FASE 3: Reanudar avance gradualmente
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3
        
        evasion_steps -= 1
        if evasion_steps == 0:
            evasion_mode = False
            print("EVASIÓN COMPLETADA, reanudando marcha")
    else:
        # CAMINO LIBRE - Avance normal con detección de pasillos/paredes
        base_speed = MAX_SPEED * 0.35
    
        # Si hay pared lateral, centrar el robot (comportamiento de seguir pared)
        lateral_correction = 0
        if left_dist < 0.07 and left_dist > 0.02:
            lateral_correction = 0.3  # Gira ligeramente a la derecha
        elif right_dist < 0.07 and right_dist > 0.02:
            lateral_correction = -0.3  # Gira ligeramente a la izquierda
    
        vL = base_speed * (1 - lateral_correction)
        vR = base_speed * (1 + lateral_correction)

    # Imprimir para depurar y ver cómo evolucionan las lecturas y la acción tomada
    print(f"FRENTE: Crudo={z_k:.3f}m | Kalman={d_est:.3f}m | Front_IR={front_dist:.3f}m", end="")
    if evasion_steps > 0:
        print(f" | EVASIÓN ({evasion_steps})", end="")
        if evasion_steps > MAX_EVASION_STEPS * 0.7:
            print(" [RETROCESO]", end="")
        elif evasion_steps > MAX_EVASION_STEPS * 0.3:
            print(" [GIRO]", end="")
        else:
            print(" [REANUDANDO]", end="")
    print()

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
