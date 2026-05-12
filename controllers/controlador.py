from controller import Robot
import math

# Creamos la instancia del robot y definimos el tiempo de muestreo
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
d_est = 0.06  # Distancia inicial estimada (m)
P = 1.0       # Incertidumbre inicial (covarianza)

# Constantes de sintonización del filtro 
# Q se escala con el movimiento (más movimiento -> mayor incertidumbre)
Q_base = 0.0005  # ruido de proceso base
R_base = 0.02    # ruido de medición base
Q = Q_base
R = R_base
GATING_THRESHOLD = 0.06  # Umbral para detectar mediciones atípicas (m)

# VARIABLES DE ALMACENAMIENTO
raw_front_data = [] # Para guardar las lecturas crudas convertidas a metros (z_k) para gráficos finales
filtered_front_data = [] # Para guardar las lecturas filtradas por EMA para gráficos finales
kalman_front_data = [] # Para guardar la estimación fusionada
time_data = [] # Para guardar los timestamps de cada lectura, útil para gráficos 

# VARIABLES AUXILIARES PARA CÁLCULO DE ODOMETRÍA
prev_left_enc = 0.0
prev_right_enc = 0.0
first_step = True
alpha = 0.3
filtered_val = 0.0

# Umbral de seguridad para la navegación (en metros)
SAFE_DIST = 0.09 # Si el Kalman estima que hay un obstáculo a 9 cm o menos, se activa la evasión forzada

# Un temporizador para obligar al robot a completar el giro, importante para evitar que se quede atascado en situaciones de obstáculos cercanos
turn_timer = 0

# VARIABLES PARA EL MODO DE EVASIÓN FORZADA
evasion_mode = False 
evasion_steps = 0
MAX_EVASION_STEPS = 45  # ~2.25 segundos de evasión
evasion_phase = "idle"
evasion_turn_dir = 1  # 1 = izquierda, -1 = derecha
front_clear_steps = 0
RELEASE_DIST = SAFE_DIST * 0.5
evasion_attempt = 0  # Contador de intentos fallidos de giro
MAX_EVASION_ATTEMPTS = 3  # Máximo de reintentos antes de forzar salida
evasion_total_steps = 0  # Tiempo total en modo evasión
MAX_EVASION_TOTAL = 200  # ~10 segundos máximo de evasión continua

# Función que convierte la lectura cruda del sensor (0-4095) a una distancia en metros, basada en una tabla de conversión o fórmula específica del sensor
def convertir_sensor_a_metros(valor_crudo):
    # Conversión suavizada y monotónica del ADC a distancia en metros
    # Evita saltos y da más peso a lecturas altas (cercanía).
    # Asumimos ADC en [0, 4095], los valores extremos se clipean
    adc = float(valor_crudo)
    if math.isnan(adc):
        return 0.12
    adc = max(0.0, min(4095.0, adc))
    # Si la lectura es muy baja, devolvemos rango máximo
    if adc < 20.0:
        return 0.12
    # Si es muy alta, devolvemos cercanía mínima
    if adc > 4000.0:
        return 0.01

    # Usamos un modelo exponencial para convertir el ADC a distancia, con parámetros elegidos para que el rango de salida sea aproximadamente [0.01, 0.12] metros, y que sea más sensible a cambios en el rango cercano
    # Parámetros elegidos empíricamente para rango ~[0.01,0.12]
    k = 6.5
    distancia = 0.01 + 0.11 * math.exp(-k * (adc / 4095.0))
    return max(0.01, min(0.12, distancia))

# Función de clamp para asegurar que las velocidades no excedan los límites físicos del robot
def clamp(x, a, b):
    return max(a, min(b, x))

# Función de actualización del filtro de Kalman para la estimación de la distancia al frente, que fusiona la medición del sensor con la predicción basada en la odometría
def kalman_update(d_prev, P_prev, z_meas, delta_d, Q_base, R_base):
    # Etapa de predicción: la distancia al frente disminuye con el avance delta_d
    d_pred = d_prev - delta_d
    d_pred = max(0.01, d_pred)

    # Escalamos Q con la magnitud del movimiento (más movimiento -> mayor incertidumbre)
    Q = Q_base * (1.0 + abs(delta_d) * 50.0)
    P_pred = P_prev + Q

    # Si la medición difiere mucho de la predicción, aumentamos R temporalmente (gating)
    innovation = z_meas - d_pred
    R = R_base
    if abs(innovation) > GATING_THRESHOLD:
        R = R_base * 8.0

    # Ganancia de Kalman
    K = P_pred / (P_pred + R)

    # Etapa de actualización
    d_upd = d_pred + K * innovation
    d_upd = max(0.01, min(0.12, d_upd))
    P_upd = (1.0 - K) * P_pred
    return d_upd, P_upd

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
    # Usamos la lectura suavizada `filtered_val` como medición para reducir ruido
    d_est, P = kalman_update(d_est, P, filtered_val, delta_d, Q_base, R_base)

    # 5. NAVEGACIÓN REACTIVA (Con evasión forzada)
    val_left = left_sensor.getValue()
    val_right = right_sensor.getValue()
    # Reutilizamos la lectura frontal máxima (más ADC -> más cercano)
    front_dist = z_k
    left_dist = convertir_sensor_a_metros(val_left) if val_left > 20 else 0.12
    right_dist = convertir_sensor_a_metros(val_right) if val_right > 20 else 0.12
    
    vL = 0.0
    vR = 0.0

    # LÓGICA DE EVASIÓN y MOVIMIENTO
    # Si el Kalman estima que hay un obstáculo a 9 cm o menos, o si el sensor frontal detecta algo muy cercano, activamos la evasión forzada
    obstacle_detected = d_est <= SAFE_DIST or front_dist <= SAFE_DIST
    if obstacle_detected and evasion_phase == "idle":
        evasion_mode = True
        evasion_phase = "backoff"
        evasion_steps = 8
        front_clear_steps = 0
        evasion_turn_dir = 1 if left_dist >= right_dist else -1
        evasion_attempt = 0
        evasion_total_steps = 0
        print(f"OBSTÁCULO DETECTADO! d_est={d_est:.3f}m, front_dist={front_dist:.3f}m")

    # Si estamos en modo de evasión, seguimos una secuencia que solo termina cuando el frente queda libre
    if evasion_mode:
        evasion_total_steps += 1
        
        if evasion_phase == "backoff":
            # Fase corta de retroceso para despegar el robot del obstáculo
            vL = -MAX_SPEED * 0.7
            vR = -MAX_SPEED * 0.7
            evasion_steps -= 1
            if evasion_steps <= 0:
                evasion_phase = "turn"
                evasion_steps = 18
                print("  Cambio a giro agresivo")

        elif evasion_phase == "turn":
            # Giro agresivo sobre su eje para cambiar de dirección rápidamente
            if evasion_turn_dir > 0:
                vL = -MAX_SPEED * 0.9
                vR = MAX_SPEED * 0.9
                print("  Giro a la izquierda")
            else:
                vL = MAX_SPEED * 0.9
                vR = -MAX_SPEED * 0.9
                print("  Giro a la derecha")

            # Si el frente ya quedó libre varios pasos seguidos, salimos de evasión
            if front_dist > RELEASE_DIST and d_est > SAFE_DIST:
                front_clear_steps += 1
            else:
                front_clear_steps = 0

            evasion_steps -= 1

            # Si seguimos bloqueados al final del giro, cambiamos el sentido e incrementamos contador
            if evasion_steps == 9 and front_clear_steps == 0:
                evasion_turn_dir *= -1
                evasion_attempt += 1
                print(f"  Intento fallido, cambiando dirección (intento {evasion_attempt})")

            if front_clear_steps >= 3:
                evasion_mode = False
                evasion_phase = "idle"
                evasion_steps = 0
                front_clear_steps = 0
                print("EVASIÓN COMPLETADA, frente despejado")
            elif evasion_steps <= 0 and evasion_attempt < MAX_EVASION_ATTEMPTS:
                # Si aún no está libre y no hemos alcanzado el máximo de intentos, reintentar
                evasion_steps = 18
            elif evasion_steps <= 0 and evasion_attempt >= MAX_EVASION_ATTEMPTS:
                # Si hemos agotado los intentos, hacer retroceso más agresivo
                evasion_phase = "emergency_backoff"
                evasion_steps = 15
                print(f"  Máximo de intentos alcanzado. Retroceso de emergencia.")

        elif evasion_phase == "emergency_backoff":
            # Retroceso agresivo para escapar de pared larga
            vL = -MAX_SPEED * 0.9
            vR = -MAX_SPEED * 0.9
            evasion_steps -= 1
            if evasion_steps <= 0:
                # Después del retroceso de emergencia, forzar salida de evasión
                evasion_mode = False
                evasion_phase = "idle"
                evasion_steps = 0
                evasion_total_steps = 0
                print("EVASIÓN FORZADA (máximo de intentos o pared larga)")

        # Timeout global: si llevamos demasiado tiempo en evasión, salir forzadamente
        if evasion_total_steps >= MAX_EVASION_TOTAL:
            evasion_mode = False
            evasion_phase = "idle"
            evasion_steps = 0
            evasion_total_steps = 0
            print("TIMEOUT DE EVASIÓN - Salida forzada tras 10 segundos")

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
    if evasion_mode:
        print(f" | EVASIÓN ({evasion_phase}:{evasion_steps} int:{evasion_attempt} t:{evasion_total_steps})", end="")
    print()

    # Clampeamos (es decir, limitamos) las velocidades a los límites físicos del robot
    vL = clamp(vL, -MAX_SPEED, MAX_SPEED)
    vR = clamp(vR, -MAX_SPEED, MAX_SPEED)

    # Aplicamos velocidades
    left_motor.setVelocity(vL)
    right_motor.setVelocity(vR)

    # 6. ALMACENAMIENTO DE DATOS PARA GRÁFICOS FINALES
    # Guardamos datos con timestamp para análisis posterior
    tiempo_seg = (step_count * TIME_STEP) / 1000.0
    time_data.append(tiempo_seg)
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
            # Usamos timestamps guardados
            tiempo_seg = time_data[i]
            print(f"{tiempo_seg:.2f},{raw_front_data[i]:.4f},{filtered_front_data[i]:.4f},{kalman_front_data[i]:.4f}")
            
        print("="*50)
        break # Rompe el bucle while y detiene el controlador por completo
