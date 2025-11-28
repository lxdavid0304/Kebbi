// IFaceControlTTSListener.aidl
package com.nuwarobotics.service.facecontrol;

// Declare any non-default types here with import statements

interface IFaceControlTTSListener {
    /**
     * Demonstrates some basic types that you can use as parameters
     * and return values in AIDL.
     */
    void onTTSComplete(String process_name);
}
