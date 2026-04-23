# Laboratorio 2 ICI4150-1: Navegación reactiva con filtrado y fusión de sensores

**Integrantes:** Ignacio Carrillo R.

**Curso:** ICI4150-1 Robótica y Sistemas Autónomos

# Introducción
La navegación es una característica fundamental para un sistema robótico autónomo. Esta propiedad depende fuertemente de la capacidad del robot para percibir y procesar su entorno, además de las acciones que realice en tiempo real en base a la información que entregan los sensores. Pero estas herramientas de percepción, cuando se usan en el mundo real, suelen presentar ruido e incertidumbre, por lo que el dato "puro" no es suficiente.

Para resolver el problema del ruido se emplean diferentes estrategias: combinación de sensores, técnicas de estimación o **algoritmos de filtrado**. Uno de los algoritmos que son útiles para filtrar el ruido de una medición es el **filtro de Kalman**.

En este laboratorio se busca aplicar el filtro de Kalman para producir estimaciones más estables bajo diferentes contextos. Con el uso de simulaciones de un robot diferencial (e-puck) en diferentes escenarios, se implementa una estrategia de navegación reactiva basada en la percepción. El sistema implementado usa la odometría de un robot, es decir, un avance estimado con encoders, y las lecturas de los sensores frontales infrarrojos en un esquema de fusión sensorial con Kalman, logrando que el robot estime con precisión la proximidad de obstáculos y mejorando sus decisiones de movimiento a diferencia de otros métodos de filtrado.
# Objetivos
