# DV Toolkit C++ Custom Modules

Collection of modules that can be used to customize the behaviour of the DV Toolkit.

### Getting started


**Configure, build and install:**
*(Make sure you have the newest dv-runtime installed)*

```sh
 mkdir build && cd build
 cmake ..
 make
```

If building in Windows:

```sh
 mkdir build && cd build
 cmake -G "MSYS Makefiles" -DCMAKE_INSTALL_PREFIX=/mingw64 ..
 make
```

**Open in an IDE**

Most IDEs will support importing a project from cmake directly.
Just import the project's `CMakeLists.txt` file.


### List of modules

**nvp_syncdavis**

A replacement input module for DAVIS cameras that allows to reset timestamps with an external synchronization signal. In dv-gui, first the button "Reset init state" must be pushed. At this point the module will wait for an external synchronization signal to start emitting data.

Currently, the only way to specify from which camera one is recording is to manually enter the serial number (eg. 00000071 or 00000172).

**nvp_sionoise**

Event filtering algorithm that performs a time thresholding on the local neighbourhood.
