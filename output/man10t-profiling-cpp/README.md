# Repetir el profiling C++ del MAN10T en RIMLS

Ejecuta los comandos desde la raíz de Fusion Force C++ en Ubuntu o WSL2. Instala los paquetes de desarrollo del [proyecto](../../README.md), Python 3.11 o posterior para dirigir la medición y `binutils` para leer los perfiles. La simulación se ejecuta enteramente en C++. La GPU NVIDIA debe responder a `nvidia-smi`, que en WSL2 también puede estar en `/usr/lib/wsl/lib/`.

```sh
cmake -S . -B build/timing -DCMAKE_BUILD_TYPE=Release -DFUSIONFORCE_BUILD_VIEWER=OFF -DBUILD_TESTING=OFF
cmake --build build/timing --parallel
g++ -std=c++20 -O3 -DNDEBUG -Iinclude tools/profile_man10t.cpp \
    build/timing/libfusionforce.a -lsqlite3 -lcrypto -lz -pthread -o build/profile_man10t

cmake -S . -B build/gprof -DCMAKE_BUILD_TYPE=Release -DFUSIONFORCE_BUILD_VIEWER=OFF \
    -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS=-pg
cmake --build build/gprof --parallel
g++ -std=c++20 -O3 -DNDEBUG -pg -DFUSIONFORCE_GPROF -Iinclude tools/profile_man10t.cpp \
    build/gprof/libfusionforce.a -lsqlite3 -lcrypto -lz -pthread -o build/profile_man10t_gprof

python3 tools/profile_man10t.py --ply /ruta/a/rimls.ply \
    --reference ../FusionForce/output/man10t-work --out output/man10t-nueva-medicion \
    --clean build/profile_man10t --instrumented build/profile_man10t_gprof --power-plan Equilibrado
```

El lanzador comprueba la huella del PLY, carga el mapa en un índice temporal y aplica las diez entradas originales. La pasada de tiempo se hace en un proceso, con calentamiento de 1 s simulado, referencia de GPU de 6 s y pausas de 2 s antes de cada ruta. Para obtener un perfil nativo por ruta, las pasadas instrumentadas reutilizan el índice y arrancan en procesos separados. Solo se activa el perfilador durante la evaluación.

El cronómetro incluye la construcción de los resultados en memoria. La carga del mapa, la escritura de los resultados, HTTP y el navegador quedan fuera de la ventana medida. Cada ruta debe completar 10 s simulados, 1001 muestras y al menos 2 m de desplazamiento XY. El lanzador compara las pasadas con y sin instrumentación y conserva el error XY respecto a la referencia Python.

Los archivos `metrics.json`, `gpu.csv`, `gprof.txt` y `profiles/` contienen mediciones y perfiles. `routes.json` y `python-baseline.json` conservan las entradas y la línea base histórica. La telemetría GPU corresponde a toda la tarjeta. Los vatios de CPU quedaron sin medir.

El paquete ZIP incluye los dos ejecutables Linux x86-64 de la medición en `bin/`. El ejecutable instrumentado permite volver a leer los perfiles sin recompilar:

```sh
gprof -b -p bin/profile_man10t_gprof profiles/*.gmon
```
