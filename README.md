# HomeMind

[Español](README.es.md)

An early prototype of a reusable autonomy capability for robots: mapping their surroundings, estimating where they are and navigating toward destinations.

## What works today

The demo runs in a browser using a scan of a real room. **The robot, its sensors and its movement are simulated.**

- **Mapping & autonomous exploration:** builds a 2D map from virtual depth readings, distinguishing free space, obstacles and unknown areas.
- **Pose estimation:** tracks the robot's position and heading from simulated motion readings, with optional sensor noise.
- **Navigation & route planning:** moves toward selected destinations in discovered space and replans when a route is blocked.
- **Visualization & export:** shows the virtual camera, sensor map and 3D reconstruction; exports maps, run summaries and AI observations.

This is a supervised prototype. Exploration can stop before covering the whole room.

**In development — object recognition:** initial tests use Muse Spark and SAM through real Meta API calls. Recognition and its connection to navigation are not fully integrated. A complete recognize-and-approach workflow is still being developed and validated.

![Virtual robot camera beside the discovered map, with a blue route and a red destination](docs/images/navigation.jpg)

*Simulated navigation toward a manually selected destination.*

![Sensor map and 3D reconstruction of the space discovered by the simulated robot](docs/images/discovered-maps.jpg)

*Discovered space shown as a sensor map and a 3D reconstruction.*

## Where we want to go

Develop a capability that different physical robots could use to move around with greater independence. Future applications could include patrols, inventory tasks and returning to a charging station when the battery is low.

We still need to complete the recognition-to-navigation integration, search for objects outside the current view, remember them across observations, and validate the system on real hardware. These task applications, battery-triggered return, physical docking and charging remain future work.

## Try it

Requires Node.js 22.12+ and a browser with WebGL 2.

```sh
npm ci
npm run dev
```

Open [the main demo](http://127.0.0.1:5173/scan-navigation.html). Select **EN** for English. The room scans are included.

For the experimental AI integration, copy `.env.example` to `.env`, add your Meta API key as `MODEL_API_KEY`, and restart. Navigation can run without a key. AI experiments require `npm run dev`; the static preview does not include the AI service.

[Delivery note](ENTREGA-2026-11-04.md) · [Project scope](docs/PROJECT_CONTEXT.md)
