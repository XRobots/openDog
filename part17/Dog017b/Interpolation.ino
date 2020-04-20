// ****** Interpolation functions **************
/** @todo remove redundant code */

int interpolationZ(int input, int duration) {
    static int savedValue;

    // check for new data
    if (input != savedValue) {
        // start interpolation (value to go to, duration)
        myRampZ.go(input, duration, LINEAR, ONCEFORWARD);
    }

    // bookmark the old value
    savedValue = input;

    return myRampZ.update();
}

int interpolationY(int input, int duration) {
    static int savedValue;

    // check for new data
    if (input != savedValue) {
        // start interpolation (value to go to, duration)
        myRampY.go(input, duration, LINEAR, ONCEFORWARD);
    }
    // bookmark the old value
    savedValue = input;

    return myRampY.update();
}

int interpolationX(int input, int duration) {
    static int savedValue;

    // check for new data
    if (input != savedValue) {
        // start interpolation (value to go to, duration)
        imyRampX.go(input, duration, LINEAR, ONCEFORWARD);
    }
    // bookmark the old value
    savedValue = input;

    return myRampX.update();
}
