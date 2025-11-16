// IFaceControlCallBack.aidl
package com.nuwarobotics.service.facecontrol;

// Declare any non-default types here with import statements

interface IFaceControlCallBack {
    /**
     * Demonstrates some basic types that you can use as parameters
     * and return values in AIDL.
     */
     void onSceneStart(String scene) ;
              void onLoadCharacter(int status);
              void onLoadSkinDone(String id);
              void onLoadSkinFail(String id, int error)  ;
              void onLoadSoundDone(int status) ;
              void onLoadJsonDone(int statusCode);
              void onLoadMotionConfig(String id, int status) ;
              void onLoadOfSound(String sound, int statusCode) ;
              void onUnLoadAllSoundsDone(int statusCode) ;
              void onMotionStart(String id) ;
              void onMotionStop(String id) ;
              void onMotionComplete(String id) ;
              void onMotionRepeat(String id) ;
              void onMotionError(String id);
              void onDecoration(String whichDecorationName, String statusMsg, int statusCode)  ;
              void onJointDecoration(String whichDecorationName, String statusMsg, int statusCode);
              void onUserTouchScreen(String body, int type, int x, int y) ;
              void on_touch_left_eye();
              void on_touch_right_eye();
              void on_touch_nose();
              void on_touch_mouth();
              void on_touch_head() ;
              void on_touch_left_edge();
              void on_touch_right_edge();
              void on_touch_bottom();
}
