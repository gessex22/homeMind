# HomeMind

[Español](README.es.md)

An early prototype exploring how a home robot could understand a request and move toward the right object.

## What works today

The demo runs in a browser using a scan of a real room. **The robot, its sensors and its movement are simulated.**

- It explores and builds a map, and you can choose a destination in the area it has discovered.
- On request, Muse Spark interprets the robot's camera image and SAM identifies a visible object. These are real calls to Meta's AI models.
- It can attempt to approach that object when it has enough distance information and a known route.

This is a supervised prototype. Exploration can stop before covering the whole room, and an AI observation does not guarantee that the robot can reach the object.

![Virtual robot camera beside the discovered map, with a blue route and a red destination](docs/images/navigation.jpg)

*Simulated navigation toward a manually selected destination.*

![Sensor map and 3D reconstruction of the space discovered by the simulated robot](docs/images/discovered-maps.jpg)

*Discovered space shown as a sensor map and a 3D reconstruction.*

## Where we want to go

A physical robot that responds to requests such as **“go to the charger”**: finding the target, reaching it and, eventually, docking.

We still need to search for objects outside the current view, remember them across observations, and validate the system on real hardware. Physical docking and charging have not been demonstrated.

## Try it

Requires Node.js 22.12+ and a browser with WebGL 2.

```sh
npm ci
npm run dev
```

Open [the main demo](http://127.0.0.1:5173/scan-navigation.html). Select **EN** for English. The room scans are included.

For object recognition, copy `.env.example` to `.env`, add your Meta API key as `MODEL_API_KEY`, and restart. Navigation can run without a key. Use `npm run dev` for the AI demo; the static preview does not include the AI service.

[Delivery note](ENTREGA-2026-11-04.md) · [Project scope](docs/PROJECT_CONTEXT.md)
