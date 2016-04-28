.. _view_command:

********************************************************************************
view
********************************************************************************

The ``view`` command can be used to visualize a point cloud using the
PCLVisualizer. The command takes a single argument, the input file name.

.. note::

    The ``view`` command is only available when PDAL is linked with PCL.

::

    $ pdal view <input>

Once the data has been loaded into the viewer, press h or H to display the
help.

::

    | Help:
    -------
              p, P   : switch to a point-based representation
              w, W   : switch to a wireframe-based representation (where available)
              s, S   : switch to a surface-based representation (where available)

              j, J   : take a .PNG snapshot of the current window view
              c, C   : display current camera/window parameters
              f, F   : fly to point mode

              e, E   : exit the interactor
              q, Q   : stop and call VTK's TerminateApp

               +/-   : increment/decrement overall point size
         +/- [+ ALT] : zoom in/out

              g, G   : display scale grid (on/off)
              u, U   : display lookup table (on/off)

        o, O         : switch between perspective/parallel projection (default = perspective)
        r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]
        CTRL + s, S  : save camera parameters
        CTRL + r, R  : restore camera parameters

        ALT + s, S   : turn stereo mode on/off
        ALT + f, F   : switch between maximized window mode and original size

              l, L           : list all available geometric and color handlers for the current actor map
        ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)
              0..9 [+ CTRL]  : switch between different color handlers (where available)

        SHIFT + left click   : select a point (start with -use_point_picking)

              x, X   : toggle rubber band selection mode for left mouse button


