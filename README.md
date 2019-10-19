# VREP INTEGRATION

1. The `v_repExtKUKA.cpp` is the plugin script that needs to be compiled in order to have a simulation.

2. In order to compile the plug in, all the files in the main directory needs to be copied in a new folder inside the programming folder of the main V-REP installation folder.

3. Then the commands to create the make file is : 

```shell
qmake -makefile v_repExtKUKA.pro
```

4. Then launch `make` to build the plug in and it generates 4 new files. 3 of them are links and the important file is the one whose name ends with "**.so.1.0.1.**" This file needs to be copied in the main installation directory of V-REP and its name needs to be changed in order to delete the "**.1.0.1**".

> :warning: the naming of the plug in is important. It needs to be "v_repExt\*" and in "\*" no other "\_" have to be present. Otherwise the V-REP main program don't load the plug in 

