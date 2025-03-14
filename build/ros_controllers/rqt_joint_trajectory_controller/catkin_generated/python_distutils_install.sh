#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/federico/catkin_ws/src/ros_controllers/rqt_joint_trajectory_controller"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/federico/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/federico/catkin_ws/install/lib/python3/dist-packages:/home/federico/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/federico/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/federico/catkin_ws/src/ros_controllers/rqt_joint_trajectory_controller/setup.py" \
    egg_info --egg-base /home/federico/catkin_ws/build/ros_controllers/rqt_joint_trajectory_controller \
    build --build-base "/home/federico/catkin_ws/build/ros_controllers/rqt_joint_trajectory_controller" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/federico/catkin_ws/install" --install-scripts="/home/federico/catkin_ws/install/bin"
