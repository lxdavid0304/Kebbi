// IFaceControlService.aidl
package com.nuwarobotics.service.facecontrol;

// Declare any non-default types here with import statements

import com.nuwarobotics.service.facecontrol.IFaceControlCallBack;
import com.nuwarobotics.service.facecontrol.IFaceControlTTSListener;
import com.nuwarobotics.service.facecontrol.achievement.Achievement;
import com.nuwarobotics.service.facecontrol.IonCompleteListener;


interface IFaceControlService {

    void setCallback(String processName, IFaceControlCallBack callback);
    void removeCallback(String processName);
    void openFaceActivity();
    void speakTTS(String processName,String talk,IFaceControlTTSListener listener);
    void showFace();
    void hideFace();
    void lockUnityWindow();
    void unLockUnityWindow();
    void setVisibility(boolean isShow) ;
    void setVisibilityToDefault() ;
    void showIdentityResult(String imgPath, String chtString, String engString, long showDuring,String speakString,IonCompleteListener listener);

    void showAchievementFile(int gifResId, long showDuring, String imgFilePath, long achvShowDuring,in Achievement achievement,String speakString);
    void showAchievementRes(int gifResId, long showDuring, int achvId, long achvShowDuring,in Achievement achievement,String speakString);
    boolean playMotion(String json,IonCompleteListener listener) ;
    void playUnityFace(String jsonFileName);
    void playVideoMotion(String fileName,IonCompleteListener listener);
    void talkPlayMotion(String json , String playMotionType, String talk);
    boolean stopMotion() ;
    boolean pauseMotion() ;
    boolean resumeMotion();
    void mouthEmotionOn(long moveSpeed,int EmotionType);
    void mouthOn(long time) ;
    void mouthOff() ;
    void changeWindowSizeAndPosition(int x, int y, int w, int h) ;
    void openEffect(String id) ;
    void closeEffect() ;
    boolean addDecoration(String name);
    void removeAllDecorations();
    void removeDecoration(String name)  ;
    void setAutoIdleAnimState(boolean stop_idle);
    List<String> getAddDecorationList()  ;
    String getCurrectEffectId()  ;
    void setVoiceListenUnityStatus(int VoiceListenStatus);
    void playTTSmotion(String speakString, String fstName, String animName, int emotionType, int motionMode, int faceMode,IonCompleteListener listener); // isMotionMode 0:ramdon 1:specific 2:withEmotion
    void pauseTTSmotion();
    void resumeTTSmotion();
    void showMedia(String type,String uri,String speak,String speakTiming,IonCompleteListener listener);
    void showFace_3rd();
    void playUnityFaceDefault();
    void playFaceAnimation(int id);
}

