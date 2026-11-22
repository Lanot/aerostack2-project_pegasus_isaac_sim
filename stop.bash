#!/bin/bash

./stop_tmuxinator.bash
./stop_tmuxinator_as2.bash
./stop_tmuxinator_ground_station.bash

# not sure about _sitl.bash if required
./stop_tmuxinator_sitl.bash
