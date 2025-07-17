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

echo_and_run cd "/home/zeb/eight-thurster/src/bluerov2/bluerov2_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zeb/eight-thurster/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zeb/eight-thurster/install/lib/python3/dist-packages:/home/zeb/eight-thurster/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zeb/eight-thurster/build" \
    "/usr/bin/python3" \
    "/home/zeb/eight-thurster/src/bluerov2/bluerov2_control/setup.py" \
     \
    build --build-base "/home/zeb/eight-thurster/build/bluerov2/bluerov2_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zeb/eight-thurster/install" --install-scripts="/home/zeb/eight-thurster/install/bin"
