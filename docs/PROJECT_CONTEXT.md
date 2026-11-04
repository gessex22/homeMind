# HomeMind — Project scope

## Purpose

Let a home robot act on everyday requests without asking the user for coordinates. The motivating example is returning to its charger.

## Current stage

HomeMind is an early browser prototype using scans of real rooms. The current demonstration covers simulated exploration, mapping and navigation to manually selected destinations. Object recognition is an integration in development, with initial real API tests using Muse Spark and SAM.

Recognition and its connection to navigation are not yet fully integrated. The complete workflow from a request to recognizing and approaching an object still needs development and validation. The existing AI observation is evidence of an initial test, not a completed navigation task.

## Next steps

1. Complete and validate recognition of a visible target and its connection to navigation, then search when the target is outside the current view.
2. Remember the same object across observations.
3. Test navigation and perception on a physical robot.
4. Develop and validate charger alignment, docking and charging.

## What the prototype establishes

It provides a place to test mapping and navigation while developing AI perception. It does not yet establish a fully integrated object-directed navigation workflow, reliable operation in a home, complete room coverage or physical docking.

The full room scan is available to the simulator and the human observer. The navigation system builds its own map from simulated sensor readings.
