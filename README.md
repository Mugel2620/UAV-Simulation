# SAR Search Pattern Simulation

This repository is a continued and modified version of the following project:

[Original repository: Drone-Project-2](https://github.com/Benj4865/Drone-Project-2)

The current version builds on the original Search and Rescue simulation and retains the existing search-pattern implementation.

## Changes

The main change in this version is:

- Updated person-in-water drift behaviour so that the person also drifts during the time since last contact, before the UAV search begins.

In the previous version, the search datum accounted for drift before the search started, while the person's drift trajectory began when the UAV search started. In this version, the person also moves during the elapsed time since last contact and then continues drifting during the UAV search.

## License

This project is based on software released under the GNU General Public License v3.0 (GPL-3.0).

The original repository can be found here:

https://github.com/Benj4865/Drone-Project-2

This repository is also distributed under the GPL-3.0 license. See the `LICENSE` file for the full license text.

## Author

Mikkel Bo Jacobsen
