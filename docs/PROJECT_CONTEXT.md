# HomeMind — contexto y alcance

## Problema

Un robot debe poder responder a «ve al cargador» o «ve al objeto» sin que una persona le indique coordenadas. El caso que originó el proyecto es un robot con NVIDIA Jetson que pueda regresar a su cargador. Llegar al cargador y conectarse físicamente son dos problemas distintos; esta demo no demuestra acoplamiento ni carga eléctrica.

El proyecto se prepara para el Meta Global AI Developer Hackathon 2026. El trabajo comienza en un laboratorio virtual y se valida por fases antes de incorporar hardware.

## Estado actual — laboratorio y primera integración real de Meta

`scan-navigation.html` ya incorpora llamadas reales a Meta Model API: Muse Spark 1.3 analiza una imagen RGB capturada desde el robot y SAM 3.1 segmenta el concepto visible. Las máscaras se muestran y exportan con los identificadores de respuesta. La primera prueba confirmó una silla de oficina y obtuvo una posición estimada con 35 retornos de profundidad virtual. La navegación a un punto de aproximación usa únicamente el mapa descubierto. La etapa posterior gira hacia el punto observado; la orientación funcional del objeto no se estima. La autenticación se lee en el backend local desde el entorno o la API key existente de Muse Code, sin devolverla al navegador.

El alcance actual es encontrar un objeto en la vista presente y acercarse. Faltan exploración guiada por la solicitud cuando el objeto no aparece, inventario semántico persistente con identidad entre vistas y orientación funcional para tareas como acoplarse a un cargador. Esta integración no incorpora todavía hardware real. `README.md` documenta las instrucciones y límites de la nueva página; las fases siguientes mantienen también la historia de la demo inicial.

### Demo inicial — keyframes y observaciones visuales

Tres tableros de color a altura de cámara permiten probar imágenes RGB reales, keyframes con pose estimada y tiempo, y detección por componentes de color. La evaluación de frontalidad del observador puede consultar la verdad del mundo; el detector y selector no. Las detecciones se repiten, no forman un inventario. No se usa SAM ni Meta.

La corrección durante la misión es un SLAM 2D experimental para el modo ruidoso: construye submapas a partir de lecturas de sensores simulados, verifica repetidamente candidatos a cierre y reconstruye observaciones históricas cuando se confirma uno. Conserva ruido, sesgos y patinaje; el control ideal mantiene la pose exacta. No garantiza corrección global ni sustituye una prueba física. Las diez pasadas virtuales siguen siendo un banco de pruebas y reciben el ajuste de corrección seleccionado al iniciar.

La prueba de escala incluye una habitación de 12 × 16 m y mapa de 20 × 20 m. El sensor ideal agrega retorno de suelo válido, distinto de lectura inválida. La interfaz permite inspeccionar posiciones de cámara y keyframes; la evaluación prioriza la mejor imagen frontal y exige por defecto un lado corto de 48 px en 480 px de ancho. Una imagen ampliada no crea detalle. La posición de una cámara no localiza un objeto.

La protección de localización es opcional: su límite mide solo tiempo autónomo sin posición confirmada, no duración total de exploración. Lecturas ambiguas no reinician el contador; tres observaciones coherentes lo restablecen. Al agotarse se reobserva sin avanzar y, si no hay confirmación, se detiene. La correa de distancia requiere confirmación para retornar. La pausa humana congela el contador y la verdad del simulador no se usa para decidir confianza.

### Fase 2 — fidelidad, visibilidad y oclusión

La fase 2 introdujo una pared interior con abertura transitable, una variante con abertura más estrecha que el robot y el escenario vacío de control. La geometría es única para físicas, sensor y vistas, pero no se entrega al planificador. La pared ocluye las lecturas; las zonas posteriores permanecen desconocidas hasta observarse. En esa fase la pose y el sensor eran ideales y aún no se había introducido SLAM.

### Fase 1 — exploración por fronteras

La fase 1 amplía el laboratorio con un sensor de distancia simulado, mapa de ocupación desconocido/libre/ocupado y exploración autónoma por posiciones alcanzables próximas a fronteras. El planificador consulta únicamente el mapa y la pose exacta del simulador; no consulta geometría real. La vista completa es para evaluación humana. La pose exacta no constituye SLAM ni localización estimada. El sensor no libera espacio ante una lectura sin retorno. La cobertura mostrada usa como denominador el dominio fijo de 8 × 8 m, no el área real de la habitación. La parada indica que no quedan nuevas posiciones de observación alcanzables con la separación entre barridos configurada; no afirma cobertura total.

### Fase 0 — laboratorio virtual

Entregar una aplicación local en navegador con:

- Una habitación vacía y un robot terrestre simulado.
- Movimiento manual hacia delante/atrás, giro y detención.
- Colisiones con las paredes teniendo en cuenta el tamaño del robot.
- Una cámara a la altura del robot, con campo de visión limitado.
- Una vista superior para el observador humano.
- Registro y exportación de posición, orientación y tiempo de simulación.
- Reinicio repetible de la prueba.

La posición exacta que ofrece el simulador sirve para validar esta fase. No constituye una implementación de SLAM, odometría ni localización con sensores reales. La cámara genera imágenes 2D de un entorno 3D; el movimiento del robot está restringido al plano del suelo.

### Criterios de aceptación

1. La vista de cámara corresponde a la posición y orientación del robot.
2. El robot no atraviesa las paredes ni permite que su cámara las cruce.
3. Soltar los controles detiene el movimiento; perder el foco no deja controles activos.
4. El recorrido se registra con unidades y un convenio de coordenadas explícitos.
5. Reiniciar devuelve al robot al estado inicial y comienza un registro nuevo.
6. La interfaz identifica la vista superior como una herramienta del observador, no como percepción del robot.

## Fases siguientes

1. Rejilla navegable y exploración por fronteras: implementadas en fase 1.
2. Verificación de fidelidad: pared y aberturas, visibilidad y oclusiones implementadas en fase 2; continuar ampliando las pruebas.
3. Tableros en paredes y selección de keyframes con sus poses: fase actual.
4. Inventario por fusión de observaciones, sin entradas duplicadas.
5. Segmentación y razonamiento semántico, selección de destino y ruta A*.
6. Escalado a más habitaciones y objetos.
7. Ampliar y validar la localización con ruido y el cierre de bucle experimental; evaluar eventualmente en un robot físico.
8. Exploración 3D con otro hardware si el proyecto lo requiere.

El orden es incremental: cada etapa necesita evidencia antes de declarar que la siguiente funciona. La demo objetivo es mapa + inventario + instrucción por lenguaje + ruta hasta el destino.

## Reglas para las futuras pruebas

- La verdad del simulador se reserva para evaluación. El robot no debe consultar el inventario o mapa completo para aparentar descubrimiento.
- Una medición de profundidad sin retorno significa desconocido, no espacio libre.
- El mismo objeto observado varias veces debe corresponder a una única entrada de inventario.
- Las tolerancias de posición y los criterios de cobertura se fijarán antes de evaluar.
- La fase 0 no requiere credenciales ni llamadas a Meta, iPhone, GPU remota, Jetson o dron.
