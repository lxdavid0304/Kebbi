package com.nuwarobotics.service.custombehavior;

// Declare any non-default types here with import statements
import com.nuwarobotics.service.custombehavior.CustomBehavior;
import com.nuwarobotics.service.custombehavior.OnBehaviorFocusChangeListener;

interface ISystemBehaviorManager {
    void register(String pkgName, CustomBehavior action);
    void unregister(String pkgName, CustomBehavior action);
    void setWelcomeSentence(in String[] sentences);
    void resetWelcomeSentence();
    void notifyBehaviorFinished();
    void beginSession();
    void endSession();
    void requestBehaviorFocus(OnBehaviorFocusChangeListener listener);
    void abandonBehaviorFocus(OnBehaviorFocusChangeListener listener);
}
