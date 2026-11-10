# Migración de FusionForce a C++

## Objetivo

Entregar en la carpeta Fusion Force C++ una versión funcionalmente equivalente a FusionForce. La carpeta hermana FusionForce proporciona la referencia del comportamiento y de los formatos.

Perfil estándar. Proyecto existente. Cinco rondas. Ambigüedad estimada 12 % frente a un umbral del 20 %.

- Contexto inicial: .omx/context/migracion-cpp-20261109T111757Z.md
- Resumen de la entrevista: .omx/interviews/migracion-cpp-20261109T112620Z.md
- Resumen adicional por exceso de contexto: no requerido.

| Dimensión | Claridad estimada |
| --- | ---: |
| Intención | 70 % |
| Resultado | 96 % |
| Alcance | 98 % |
| Restricciones | 88 % |
| Criterios de aceptación | 94 % |
| Contexto | 96 % |

## Alcance

- Portar el código Python propio necesario para el visor web, la importación de nubes, la colocación, el motor físico, la estabilidad, la reproducción y la persistencia y exportación.
- Portar los nodos e integración con ROS 2 y Gazebo, los componentes de aprendizaje y sus flujos de entrenamiento, evaluación y optimización, y las utilidades y pruebas propias que forman parte de esos flujos.
- Mantener la interfaz web y reutilizar HTML, CSS, JavaScript, configuraciones y recursos cuando preserven el comportamiento existente.
- Separar el código mantenido de cachés, perfiles generados, copias de trabajo y resultados históricos. Utilizar referencias reproducibles para las comprobaciones de paridad.

## Restricciones y límites de decisión

- Linux es la plataforma objetivo. WSL2 es un entorno válido desde Windows.
- La compilación, ejecución, lanzamiento y pruebas de la entrega deben funcionar sin requerir ni invocar Python. ROS 2 puede instalar herramientas Python que FusionForce no use.
- La generación de referencias durante el desarrollo puede ejecutarse por separado en el proyecto Python original. Las pruebas del producto C++ deben poder usar las referencias ya generadas.
- Se pueden decidir la estructura C++, el sistema de compilación, las bibliotecas nativas y las tolerancias justificadas. Evaluar la cadena completa de dependencias para cada flujo antes de seleccionarlas.
- Windows nativo, rediseño del visor, cambios de contratos, optimizaciones que alteren decisiones y sustitución de algoritmos por aproximaciones que incumplan la paridad quedan fuera del encargo.

## Criterios de aceptación

1. Compilar desde una copia limpia y ejecutar las pruebas en Linux o WSL2 con un procedimiento reproducible que prescinda de Python.
2. Verificar los flujos del visor de extremo a extremo. Incluir PLY, colocación, simulación manual y con ruta, progreso, estabilidad, guardado y reapertura de proyectos existentes, reproducción y exportación.
3. Verificar los contratos de API, mensajes ROS 2, archivos .fusionforce y CSV, formatos de datos y compatibilidad de configuraciones y artefactos de aprendizaje usados por el original. Mantener formatos y decisiones observables.
4. Validar escenarios de Gazebo, nodos ROS 2, entrenamiento, evaluación y optimización con entradas y configuraciones equivalentes.
5. Comparar cantidades numéricas por magnitud con tolerancias documentadas y justificadas a partir de las pruebas y de la referencia. Incluir resultados discretos como llegada a destino, falta de cobertura y errores. Usar MAN10T/RIMLS como uno de los casos de referencia.
6. Comprobar que ningún paso requerido de compilación, ejecución o pruebas invoque Python, incluidas herramientas transitivas. Registrar resultados por flujo y plataforma.

## Evidencia y decisiones de la entrevista

- DESIGN.md y viewer/server.py delimitan el simulador web. Los manifiestos package.xml, setup.py y los lanzadores muestran la dependencia actual de Python en ROS 2. Los scripts y módulos de aprendizaje establecen un segundo frente de migración.
- viewer/tests comprueba varios resultados con tolerancia. output/man10t-work aporta trayectorias y referencias históricas. La entrevista fija el criterio que debe regir la comparación entre implementaciones.
- La frase «todo el código» se puso a prueba con el caso concreto de un simulador C++ que aún depende de un lanzador Python. Se excluyó esa dependencia. Se admitió Python instalado como parte del entorno, siempre que FusionForce pueda prescindir de él.
- Las decisiones de arquitectura y las tolerancias concretas quedan para planificación, sujetas a los criterios anteriores. Confirmar en esa fase versiones de Linux, ROS 2, Gazebo, compilador y herramientas de aprendizaje mediante comprobaciones de compatibilidad.
