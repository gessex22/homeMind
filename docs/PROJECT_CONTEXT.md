# HomeMind — Project scope

## Purpose

Develop a reusable autonomy capability that helps robots map their surroundings, estimate their position and navigate toward destinations with less manual guidance.

Returning to a charger when the battery is low is one motivating use case. The same capability could support future patrol or inventory systems. These are intended applications; the current prototype tests the underlying navigation capability in simulation.

## Current stage

HomeMind is an early browser prototype using 3D room scans created with Polycam, an external tool. These prepared environments simplify study and development. The scans are loaded from local files; the demo does not require a running Polycam service.

The current demonstration covers simulated exploration, navigation-map building and navigation to manually selected destinations. HomeMind builds that navigation map from virtual sensor readings. It has not generated the source room reconstruction from a robot's own physical sensors. Object recognition is an integration in development, with initial real API tests using Muse Spark and SAM.

Recognition and its connection to navigation are not yet fully integrated. The complete workflow from a request to recognizing and approaching an object still needs development and validation. The existing AI observation is evidence of an initial test, not a completed navigation task.

## Next steps

1. Complete and validate recognition of a visible target and its connection to navigation, then search when the target is outside the current view.
2. Remember the same object across observations.
3. Test on a physical robot and develop environment capture and reconstruction from its own sensors, so a prepared external room scan is no longer needed for that workflow.
4. Develop task integrations, including battery-triggered return, charger alignment, docking and charging.

## What the prototype establishes

It provides a place to test mapping and navigation while developing AI perception. It does not yet establish a fully integrated object-directed navigation workflow, reliable operation in a home, complete room coverage or physical docking.

The Polycam room scan is available to the simulator and the human observer. The navigation system builds its own map from simulated sensor readings. The 3D discovered view is a simple representation of that navigation map, rather than a new textured reconstruction of the original room.
