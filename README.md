# Modna Lagra

- The ripeness of avocados is assessed automatically by scanning them with a
  multispectral camera.
- Avocados that are ready to eat are split into firm ripe and soft ripe,
  and made available to customers in separate sections on top of the machine.
- Other avocados are distributed between different temperature zones for
  ripening, based on their ripeness and the expected customer demand for firm
  and soft ripe avocados going forward. This is a future plan, however, and we
  don't need to take it too much into account.
- In this module we don't need to worry about all the details of the scanning;
  that is handled elsewhere.

Below are more details on a suggested implementation. None of this is a
strict requirement that has to be followed.
- The machine could, for example, be about 70 cm wide and deep, and 80 cm high.
  - This is very approximate and can be adjusted as appropriate.
  - The back of the machine can be taller, but the fruit must not sit higher
    than customers can reach.
- I want three different sections of avocados on top: soft ripe, firm ripe,
  and ripe in a few days.
- I want about 3–8 shelves inside the machine for storing avocados.
  Each shelf should hold about 20–50 avocados.
- I want to use a linear actuator as an elevator, with an X and a Z axis, so
  it can also control the position on the shelf. The elevator moves the
  avocado between the shelves, including the shelves on top that are
  accessible to customers.
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
