# HomeMind — Project scope

## Purpose

Let a home robot act on everyday requests without asking the user for coordinates. The motivating example is returning to its charger.

## Current stage

HomeMind is an early browser prototype using scans of real rooms. It combines simulated exploration and navigation with real Meta AI calls to interpret a camera image and identify a visible object.

The user starts exploration, requests an analysis and chooses whether to approach a detected object. Movement depends on the space already mapped and the distance information available. A detection alone does not prove that the object can be reached.

## Next steps

1. Search for a requested object when it is outside the current view.
2. Remember the same object across observations.
3. Test navigation and perception on a physical robot.
4. Develop and validate charger alignment, docking and charging.

## What the prototype establishes

It provides a place to test the connection between seeing an object and moving toward it. It does not yet establish reliable operation in a home, complete room coverage or physical docking.

The full room scan is available to the simulator and the human observer. The navigation system builds its own map from simulated sensor readings.
