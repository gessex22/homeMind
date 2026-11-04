# Corrección durante una sola misión — validación

Implementación experimental propia en JavaScript. No usa poses reales, paredes del simulador, posiciones de tableros o resultados de otras pasadas para reconocer un regreso. La base y su pose inicial sí son conocidas. Conserva el modelo de ruido y sesgos existente.

## Mecanismo

Se registran lecturas calibradas y poses estimadas en nodos locales. Las referencias reúnen 25 lecturas y se buscan después de al menos 12 segundos y 4 metros de recorrido estimado. La búsqueda está limitada al entorno de la estimación actual: no resuelve un robot trasladado a un lugar arbitrario. Se exige una coincidencia distinguible de paredes, se afina contra superficies de las lecturas originales y se comprueba que los rayos no crucen paredes conocidas. Una coincidencia necesita tres comprobaciones separadas por al menos 0,5 segundos.

Al aceptarla se añade una restricción entre el nodo histórico y el actual. Un optimizador disperso SE(2) distribuye el ajuste conservando fijo el primer nodo. Si no reduce el coste, deja un residuo excesivo o desplaza demasiado las poses, se rechaza. Al aceptar, se reconstruyen la rejilla, los contactos y todos los submapas retenidos desde las observaciones corregidas. Se invalidan ruta y confirmación de posición; se reobserva antes de volver a avanzar. El tiempo ya acumulado sin localización no se borra.

## Comparación reproducible

Comando: `npm run benchmark:slam -- docs/slam-benchmark.json`. Mismos parámetros de sensores y semillas, con corrección histórica desactivada/activada. La corrección cambia acciones posteriores: las trayectorias y duraciones pueden ser distintas. Los errores y coberturas siguientes pertenecen al observador y nunca alimentan el algoritmo.

| Escena | Semilla | Error final sin/con (m) | Pared observada sin/con | Cierres | Resultado con corrección |
| --- | ---: | ---: | ---: | ---: | --- |
| Abertura 1 m | 7 | 0,2611 / 0,0669 | 93,45 % / 100 % | 1 | Limitado |
| Abertura 1 m | 7926 | 0,0998 / 0,0998 | 95,52 % / 95,52 % | 0 | Bloqueado |
| Casa | 7 | 0,0223 / 0,0223 | 18,78 % / 18,78 % | 0 | Bloqueado |
| Casa | 7926 | 0,2746 / 0,3949 | 80 % / 52,45 % | 1 | Limitado |
| Sala grande | 7 | 1,2500 / 1,2500 | 18,92 % / 18,92 % | 0 | Bloqueado |
| Sala grande | 7926 | 1,5040 / 1,5040 | 47,90 % / 47,90 % | 0 | Bloqueado |

Las seis misiones con corrección terminaron sin colisiones ni pasos fuera de la huella libre conocida. Ninguna se presenta como cobertura completa. En la casa con semilla 7926, el cierre reduce el error instantáneo de posición de 0,275 a 0,114 m, pero la misión posterior termina con más deriva y menos cobertura; esto impide afirmar una mejora general. En la sala grande falta un regreso reconocible dentro del límite de búsqueda antes de bloquearse.

Los resultados detallados y la evaluación instantánea de cada cierre están en `slam-benchmark.json`. Las semillas repiten el experimento dentro del mismo entorno; diferencias numéricas entre motores pueden cambiar la ruta.

## Pruebas automatizadas

`npm test` incluye transformaciones y optimización del grafo con ancla fija, reconocimiento de esquina con ruido, rechazo de pared uniforme/desconocido/rayos obstruidos, ausencia de cierres por permanecer quieto, confirmaciones repetidas, reconstrucción de historial y submapas, conservación de contactos, serialización, invalidación de localización sin borrar su reloj y límite de memoria explícito. Se conserva la evaluación previa sin corrección para comparar y detectar regresiones.

Pendiente: mejorar recuperación antes del bloqueo, calibración de incertidumbre y cobertura en espacios grandes. Esta implementación no equivale a un sistema de navegación comercial ni está validada en hardware.
