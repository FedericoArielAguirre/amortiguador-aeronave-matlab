---
config:
  layout: dagre
---
flowchart TD
 subgraph subGraph0["I) Ajuste de Curva"]
        D["Ingresar Datos Experimentales<br>(velocidad, fuerza)"]
        E{"Bucle para cada Grado de Polinomio"}
        F["Realizar Ajuste Polinomial (polyfit)"]
        G["Calcular Error RMS"]
        H["Graficar Datos y Curva Ajustada"]
        I["Seleccionar Mejor Ajuste (Menor Error)"]
  end
 subgraph subGraph1["II) Búsqueda de Parámetros k y c"]
        J["Definir Rangos de Búsqueda para k y c"]
        K{"Bucle a través de los valores de k"}
        L{"Bucle a través de los valores de c"}
        M["Aproximar Fuerza con k y c de prueba"]
        N["Calcular Error vs. Curva Objetivo"]
        O{"¿Error actual &lt; mejor error?"}
        P["Actualizar Mejor k, c y Error"]
        Q["Guardar k y c Óptimos"]
  end
 subgraph subGraph2["III) Resolver EDO"]
        R["Inicializar Simulación<br>(t_final, dt, condiciones iniciales u(0), u_dot(0))"]
        S["Establecer Parámetros del Sistema<br>(k y c óptimos, precarga)"]
        T{"Bucle a través de cada Paso de Tiempo dt"}
        U["Calcular Aceleración:<br>ü = (F_ext - c*u̇ - k*u) / m"]
        V["Actualizar Velocidad y Desplazamiento<br>(Método de Euler)"]
        W["Guardar Resultados Completos de la Simulación<br>(Desplazamiento, Velocidad)"]
  end
 subgraph subGraph3["IV) Verificación y Análisis"]
        X["<b>Verificación de Restricciones</b>"]
        X1["Calcular Fuerza Máx., Carrera Máx. y Tiempo de Amortiguación al 98% de los resultados"]
        Y["<b>IV) Análisis de Energía</b>"]
        Y1["Calcular Energía Cinética, Potencial, Disipada y Total"]
        Z["<b>Graficación</b>"]
        Z1["Generar Gráficos:<br>- Desplazamiento, Velocidad, Fuerza vs. Tiempo<br>- Evolución de la Energía<br>- Diagrama de Fase"]
  end
 subgraph subGraph4["V) Propuesta de Mejora"]
        AA@{ label: "Crear un nuevo rango para la constante de amortiguación 'c'" }
        AB@{ label: "Bucle a través de los nuevos valores de 'c'" }
        AC["Ejecutar una nueva simulación"]
        AD{"¿La simulación cumple las restricciones de fuerza y carrera?"}
        AE["Calcular Tiempo de Amortiguación al 98%"]
        AF["Resultado inválido (tiempo = inf)"]
        AG@{ label: "Encontrar 'c' que da el mínimo tiempo de amortiguación válido" }
  end
 subgraph subGraph5["Reporte Final"]
        AH["Mostrar Resumen Final del Diseño"]
        AI["Mostrar parámetros finales (k, c) y verificación de restricciones (Cumple/No Cumple)"]
        AJ@{ label: "Presentar Propuesta de Mejora con 'c' optimizado y mejor tiempo de amortiguación" }
  end
    A("Inicio") --> B["Inicializar Datos del Problema y Constantes<br>(P_max, v0, f_max, carrera_max, etc.)"]
    B --> C["Calcular Masa sobre Tren de Nariz (m)"]
    C --> D
    D --> E
    E --> F
    F --> G
    G --> H
    H --> E
    E -- Fin del Bucle --> I
    I --> J
    J --> K
    K --> L
    L --> M
    M --> N
    N --> O
    O -- Sí --> P
    P --> L
    O -- No --> L
    L -- Fin Bucle c --> K
    K -- Fin Bucle k --> Q
    Q --> R
    R --> S
    S --> T
    T --> U
    U --> V
    V --> T
    T -- Fin del Bucle --> W
    W --> X
    X --> X1
    X1 --> Y
    Y --> Y1
    Y1 --> Z
    Z --> Z1
    Z1 --> AA
    AA --> AB
    AB --> AC
    AC --> AD
    AD -- Sí --> AE
    AD -- No --> AF
    AE --> AB
    AF --> AB
    AB -- Fin del Bucle --> AG
    AG --> AH
    AH --> AI
    AI --> AJ
    AJ --> AK("Fin")
    AA@{ shape: rect}
    AB@{ shape: diamond}
    AG@{ shape: rect}
    AJ@{ shape: rect}
