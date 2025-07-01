# Laboratorio Integrador con MATLAB - Diseño de Amortiguador para Aeronave

## 📋 Descripción del Proyecto

Este repositorio contiene la solución completa para el diseño y análisis de un amortiguador destinado al tren de nariz de una aeronave. El proyecto abarca desde el modelado matemático hasta la simulación y optimización del sistema de amortiguación.

### 🎯 Objetivo Principal

Diseñar un amortiguador que cumpla con una respuesta dinámica específica de fuerza en función de la velocidad, basándose en valores medios de fuerza durante el primer segundo de desplazamiento a velocidad constante en el rango de 200-900 mm/s.

## 🛩️ Especificaciones de la Aeronave

- **Modelo de referencia**: Socata TBM700
- **Peso máximo de aterrizaje**: 2900 kg
- **Carga sobre tren de nariz**: 20% del peso total
- **Velocidad inicial de desplazamiento**: 0.75 m/s

## 📊 Condiciones de Borde

| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| Fuerza máxima transmitida (f) | 7500 | N |
| Carrera máxima del amortiguador | 350 | mm |
| Tiempo máximo de reducción 98% | 2.5 | s |
| Precarga inicial sugerida | 2750 | N |

## 🔧 Estructura del Repositorio

```
📦 amortiguador-aeronave-matlab/
├── 📁 src/
│   ├── 📄 main.m                          # Código principal MATLAB
│   ├── 📄 ajuste_curva.m                  # Ajuste de curva por mínimos cuadrados
│   ├── 📄 optimizacion_kc.m               # Optimización de parámetros k y c
│   ├── 📄 diferencias_finitas.m           # Resolución por diferencias finitas
│   ├── 📄 energia_disipada.m              # Cálculo de energía disipada
│   └── 📄 mejora_sistema.m                # Propuesta de mejora del sistema
├── 📁 docs/
│   ├── 📄 informe.tex                     # Documento LaTeX completo
│   ├── 📄 informe.pdf                     # Informe compilado
│   └── 📁 figuras/                        # Gráficos generados
├── 📁 flowcharts/
│   ├── 📄 diagrama_flujo_principal.md     # Diagrama Mermaid
│   ├── 📄 diagrama_flujo_optimizacion.md  # Diagrama Mermaid
│   ├── 📄 diagrama_flujo_dif_finitas.md   # Diagrama Mermaid
│   └── 📄 diagrama_flujo_mejora.md        # Diagrama Mermaid                    
└── 📄 README.md                           # Este archivo
```

## 🧮 Metodología de Resolución

### 1. **Ajuste de Curva** 
- Implementación del método de mínimos cuadrados
- Representación matemática de la relación fuerza vs velocidad
- Cálculo de errores asociados

### 2. **Optimización de Parámetros**
- Método iterativo para encontrar combinaciones óptimas de k y c
- Comparación con curva objetivo
- Criterios de aceptabilidad definidos

### 3. **Resolución Numérica**
- Aplicación del Método de Diferencias Finitas
- Ecuación diferencial: `m*ü + c*u̇ + k*u = -5689.8 + precarga`
- Verificación de restricciones

### 4. **Análisis Energético**
- Cálculo de energía disipada: `E = ∫ c*u̇² dt`
- Verificación de consistencia física
- Evolución temporal de variables

### 5. **Optimización del Sistema**
- Análisis de alternativas de mejora
- Minimización del tiempo de amortiguación
- Respeto de todas las restricciones

## 🚀 Ejecución del Código

### Requisitos Previos
- MATLAB R2020a o superior

### Instrucciones de Uso

1. **Clonar el repositorio**:
   ```bash
   git clone https://github.com/FedericoArielAguirre/amortiguador-aeronave-matlab.git
   cd amortiguador-aeronave-matlab
   ```

2. **Ejecutar análisis completo**:
   ```matlab
   % En MATLAB, navegar al directorio del proyecto
   cd('src/')
   main  % Ejecuta el análisis completo
   ```

3. **Ejecutar módulos individuales**:
   ```matlab
   % Ajuste de curva
   ajuste_curva
   
   % Optimización de parámetros
   optimizacion_kc
   
   % Resolución numérica
   diferencias_finitas
   
   % Análisis energético
   energia_disipada
   
   % Propuesta de mejora
   mejora_sistema
   ```

## 📈 Resultados Esperados

El código generará:

- **Gráficos de ajuste de curva** con coeficientes y errores
- **Evolución temporal** de:
  - Posición
  - Velocidad  
  - Energía cinética
  - Energía disipada
- **Análisis de cumplimiento** de restricciones
- **Propuesta optimizada** del sistema
- **Tablas de resultados** con parámetros finales

## 📋 Diagrama de Flujo del Algoritmo

Los diagramas de flujo detallados están disponibles en formato Mermaid en el directorio `flowcharts/`. Incluyen:

- Flujo principal del algoritmo
- Proceso de optimización iterativa
- Metodología de diferencias finitas
- Algoritmo de mejora del sistema

## 📄 Documentación Técnica

El informe técnico completo está disponible en LaTeX (`docs/informe.tex`) e incluye:

- **Memoria de cálculo** detallada
- **Justificación teórica** de la metodología
- **Análisis de resultados** con gráficos
- **Conclusiones** y recomendaciones
- **Propuesta de mejora** fundamentada

## 🔍 Validación de Resultados

### Criterios de Aceptación

- ✅ **Fuerza máxima** ≤ 7500 N
- ✅ **Carrera máxima** ≤ 350 mm  
- ✅ **Tiempo de amortiguación** ≤ 2.5 s
- ✅ **Error de ajuste** < 5%
- ✅ **Consistencia física** verificada

### Métricas de Performance

| Métrica | Objetivo | Resultado |
|---------|----------|-----------|
| Error RMS ajuste | < 5% | - |
| Tiempo simulación | < 3.0 s | - |
| Reducción amplitud | 98% | - |
| Cumplimiento restricciones | 100% | - |

## 🛠️ Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el repositorio
2. Crear una rama para la nueva funcionalidad
3. Realizar commits descriptivos
4. Abrir un Pull Request

## 📞 Contacto

Para consultas técnicas o reportar problemas:
- Crear un Issue en GitHub
- Revisar la documentación en `docs/`

## 📚 Referencias

- Especificaciones técnicas Socata TBM700
- Teoría de sistemas de amortiguación aeronáutica
- Métodos numéricos para ecuaciones diferenciales
- Optimización de sistemas dinámicos

## 📝 Licencia

Este proyecto es de uso académico y está disponible bajo licencia MIT.

---

**Nota**: Este proyecto forma parte del Laboratorio Integrador de MATLAB para la asignatura Introducción a la Programación y Analísis Numérico del departamento de Ciencias Básicas de la Facultad de Ingeniería de la Universidad Nacional de La Plata. Los resultados son válidos únicamente para fines educativos.
