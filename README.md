# Laboratorio 2 ICI4150-1: Navegación reactiva con filtrado y fusión de sensores

**Integrantes:** Ignacio Carrillo R.

**Curso:** ICI4150-1 Robótica y Sistemas Autónomos

# Introducción
La **navegación** es una característica fundamental para un sistema robótico autónomo. Esta propiedad depende fuertemente de la capacidad del robot para percibir y procesar su entorno, además de las acciones que realice en tiempo real en base a la información que entregan los sensores. Pero estas herramientas de percepción, cuando se usan en el mundo real, suelen presentar ruido e incertidumbre, por lo que el dato "puro" no es suficiente.

Para resolver el problema del ruido se emplean diferentes estrategias: combinación de sensores, técnicas de estimación o **algoritmos de filtrado**. Uno de los algoritmos que son útiles para filtrar el ruido de una medición es el **filtro de Kalman**.

En este laboratorio se busca aplicar el filtro de Kalman para producir estimaciones más estables bajo diferentes contextos. Con el uso de simulaciones de un robot diferencial (e-puck) en diferentes escenarios, se implementa una estrategia de navegación reactiva basada en la percepción. El sistema implementado usa la odometría de un robot, es decir, un avance estimado con encoders, y las lecturas de los sensores frontales infrarrojos en un esquema de fusión sensorial con Kalman, logrando que el robot estime con precisión la proximidad de obstáculos y mejorando sus decisiones de movimiento a diferencia de otros métodos de filtrado.

# Objetivos
El objetivo central del laboratorio es implementar un sistema simple de navegación reactiva con un e-puck en la plataforma de simulación Webots, usando sensores de distancia y encoders, aplicando un filtrado de Kalman para estimar la distancia de obstáculos frontales y **mejorar la toma de desiciones y navegación del robot**.

Los objetivos específicos a considerar son:
- Adquirir datos, con cierta frecuencia de muestreo, de los sensores frontales, laterales y encoders de ruedas del e-puck.
- Estimar el movimiento y avance del robot mediante odometría (giro angular de los encoders) para un modelo de predicción.
- Implementar un filtro de Kalman con sus dos etapas: predicción (movimiento del robot) y corrección (medición del entorno).
- Desarrollar reglas de decisión usando la distancia calculada por Kalman para estimar cuando debe avanzar o girar ante un obstáculo.
- Graficar y realizar una comparación de los datos crudos del sistema, las señales con un filtro simple aplicado y la estimación final de Kalman.

# Descripción del robot y entorno de simulación
Las pruebas realizadas en este laboratorio se realizaron en la plataforma de simulación **Webots**, un entorno virtual que permite crear, implementar y diseñar escenarios de simulación, robots y controladores con código.

El robot móvil que se utilizó en las pruebas fue un **e-puck**, un sistema que cuenta con dos ruedas, sensores infrarrojos y encoders de alta precisión. 

Respecto a los sensores del e-puck, se activaron los siguientes dispositivos con tal de permitir la percepción y navegación reactiva del robot:
- **Sensores de distancia:** Se utilizaron para detectar la presencia de obstáculos. Estos se dividen en: **sensores frontales** (ubicados en la parte frontal del robot, los cuales actuaron como la principal fuente de datos para el filtrado de Kalman y se determinan como `ps0` y `ps7` en el controlador) y **sensores laterales** (se posicionan a los lados del e-puck, se utilizaron para decidir la dirección del giro evasivo y se determinan, para el lado derecho e izquierdo, como `ps2` y `ps5`).
- **Encoders de ruedas:** Se usaron los encoders, o sensores de posición, de la rueda izquierda y derecha. Los encoders permiten **medir el desplazamiento angular** en radianes, lo que permite calcular el avance lineal del e-puck gracias a la relación $s = r \theta$, donde $s$ es el desplazamiento lineal, $r$ es el radio de la rueda y $\theta$ es el desplazamiento angular medido por el encoder. Los datos del encoder **son valiosos para la fase de predicción del filtro de Kalman**.

Se estableció una **frecuencia de muestreo** para los sensores con tal de asegurar estabilidad en el filtrado de Kalman y obtener una respuesta de control adecuada. Los valores establecidos son:
- **Tiempo de muestreo ($T_s$):** 0.05 segundos o 50 ms.
- **Frecuencia de muestreo ($f_s$):** 20 Hz.

Con esta configuración se reducen los errores de la odometría y permite al controlador procesar las señales, actualizar sus estimaciones y tomar decisiones sincronizadas con el entorno virtual.

# Desarrollo e implementación
A continuación entraremos en detalle sobre el proceso de diseño del controlador, desde una perspectiva teórica y el procesamiento de señales simples hasta la implementación completa de un algoritmo de fusión sensorial.

## Estimación del avance con encoders

# Resultados y desempeño en escenarios de prueba

# Conclusiones

# ¿Cómo ejecutar la simulación en Webots?
