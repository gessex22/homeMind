# HomeMind

[English](README.md)

Un prototipo inicial para explorar cómo un robot doméstico podría entender una petición y acercarse al objeto indicado.

## Qué hace hoy

La demo funciona en el navegador sobre el escaneo de una habitación real. **El robot, sus sensores y su movimiento son simulados.**

- Explora y construye un mapa; puedes elegir un destino en la zona descubierta.
- Cuando lo solicitas, Muse Spark interpreta la imagen de su cámara y SAM identifica un objeto visible. Las llamadas a los modelos de Meta son reales.
- Puede intentar acercarse al objeto si dispone de suficiente información de distancia y una ruta conocida.

Es un prototipo supervisado. La exploración puede detenerse sin cubrir toda la habitación y detectar un objeto no garantiza poder llegar a él.

![Cámara del robot virtual junto al mapa descubierto, con la ruta azul y el destino rojo](docs/images/navigation.jpg)

*Navegación simulada hacia un destino elegido manualmente.*

![Mapa de sensores y reconstrucción 3D del espacio descubierto por el robot simulado](docs/images/discovered-maps.jpg)

*Espacio descubierto representado como mapa de sensores y reconstrucción 3D.*

## Objetivo final

Un robot físico que responda a peticiones como **«ve al cargador»**: encontrar el objetivo, llegar hasta él y, más adelante, acoplarse.

Falta buscar objetos fuera de la vista actual, recordarlos entre observaciones y validar el sistema con hardware real. Todavía no se ha demostrado acoplamiento ni carga física.

## Probarlo

Requiere Node.js 22.12+ y un navegador con WebGL 2.

```sh
npm ci
npm run dev
```

Abre [la demo principal](http://127.0.0.1:5173/scan-navigation.html). Puedes elegir **ES** o **EN**. Los escaneos están incluidos.

Para reconocer objetos, copia `.env.example` a `.env`, añade tu clave de Meta en `MODEL_API_KEY` y reinicia. La navegación funciona sin clave. La demo de IA necesita `npm run dev`; la vista estática no incluye ese servicio.
