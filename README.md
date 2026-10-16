# Modna Lagra

A DIY-prototype machine that scans avocados with a multispectral camera,
stores them inside on shelves, and moves them with an elevator at the back
to customer-facing sections on top, sorted by ripeness. This repository holds
a parametric CAD concept model of the machine (build123d) and its 3D-printed
parts.

## Getting started

Requires [uv](https://docs.astral.sh/uv/) (Python 3.14).

```sh
uv run python src/modna/lagra/avocado_manager.py   # whole machine
uv run python src/modna/lagra/rigid_chain.py       # fork chain parts
```

The first writes `avocado_manager.step` and prints a capacity and mechanism
report. It also shows the model in the VS Code OCP CAD Viewer if its panel is
open. The second writes printable chain STLs and a bench-test STEP to
`rigid_chain_print/`.

## Repository layout

- `src/modna/lagra/avocado_manager.py`: concept model of the whole machine.
  All dimensions are parameters at the top of the script.
- `src/modna/lagra/rigid_chain.py`: the fork's 3D-printable rigid chain.
- `camera-holder/`: 3D-printed Raspberry Pi camera + LED holder (see its
  README). It's an older design made before the current machine model and
  probably won't fit the imaging module as it is.

## Requirements

The original brief. None of the implementation suggestions are strict
requirements.

- The ripeness of avocados is assessed automatically by scanning them with a
  multispectral camera.
- Avocados that are ready to eat are split into firm ripe and soft ripe,
  and made available to customers in separate sections on top of the machine.
- Other avocados are distributed between different temperature zones for
  ripening, based on their ripeness and the expected customer demand for firm
  and soft ripe avocados going forward. This is a future plan, however, and we
  don't need to take it too much into account.

Suggested implementation:
- The machine could, for example, be about 70 cm wide and deep, and 80 cm high.
  - This is very approximate and can be adjusted as appropriate.
  - The back of the machine can be taller, but the fruit must not sit higher
    than customers can reach.
- I want three different sections of avocados on top: soft ripe, firm ripe,
  and ripe in a few days.
- I want about 3–8 shelves inside the machine for storing avocados.
  Each shelf should hold about 20–50 avocados.
- An elevator with an X and a Z axis moves the avocado between the shelves,
  including the shelves on top that are accessible to customers. The X axis
  also sets the position on the shelf.
- The elevator is at the back of the machine.
- The avocados can be put in and taken out with a fork that moves in the
  Y direction.
- The elevator doesn't need to hold more than one avocado, and shouldn't
  need any gripping mechanism.
- I want each shelf to be divided into lanes. One could, for example,
  keep avocados of the same ripeness in each lane. A LIFO solution is
  acceptable, since whole lanes will likely be filled and emptied close
  together in time.
- The shelves must be food-safe and should be easy to clean.
- The store staff load the machine from the front.
- Ideally the machine should generalise to other fruit, such as mango,
  tomato, kiwi and pear.
- I want the system to be relatively cheap and simple, ideally so that a
  prototype can be built as a DIY project, while still being able to scale
  up to a larger project later.

## Current design

What the concept model currently does. Exact sizes and capacity follow the
parameters; run the model to see the numbers in its report.

- **Housing:** about 60 × 70 × 80 cm (W × D × H), with a technical
  compartment at the bottom and a raised back over the elevator shaft.
- **Storage shelves:** each level is a bed of parallel stainless rods running
  front to back. There are no fixed dividers: the software chooses lanes to
  suit each fruit's width. Fruit are stored LIFO along each lane, with a gap
  between them, so they never press on each other.
- **Loading drawer:** the top storage level is a drawer that staff pull out
  at the front and fill from above. It has fixed lanes and a rear gate that
  lifts clear of the fork when the drawer closes.
- **Customer section:** sloped trays on top, one per ripeness section. The
  slope is set from the measured friction between fruit and tray.
- **Elevator:** an X/Z carriage at the back carries a fork. The fork is pushed
  into a lane by a 3D-printed rigid chain, which is rigid downwards, curls
  up into a magazine, and has a keel that runs between the shelf rods. Sag at
  full reach is corrected in software.
- **Imaging:** a swappable imaging module in its own drawer under the loading
  drawer. It spins the fruit on rollers so a side camera can image all sides.
  The camera sits on a motorised slide that varies its distance. The whole
  roller bed rests on a load cell, so each fruit is weighed.
- **Sensors:** a distance sensor in the fork's pusher face, a rod finder and
  a light beam on the carriage, a sensor that reports the loading drawer
  closed, and a door interlock.
- **Temperature zones** are left out for now.

## Open questions

- **Long fork reach:** the fork reaches about 45 cm into a lane, which drives
  the chain magazine and the tall back. Alternatives under consideration:
  - lanes that move, driven by the visiting carriage
  - spring pushers that index each lane's queue at the back, which conflicts
    with keeping fruit from pressing on each other
- **Imaging module:** roller grip on the skin, fruit walking along the rollers
  while spinning, the fruit's ends only being seen at a slant, and the
  drawer's cable loop. Stray light is deliberately deferred: there is a
  solution in mind, but it isn't modelled yet.
- **Temperature zones:** how to add per-level zones later.
