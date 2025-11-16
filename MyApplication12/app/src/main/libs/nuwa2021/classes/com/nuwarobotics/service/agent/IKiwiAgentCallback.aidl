package com.nuwarobotics.service.agent;
interface IKiwiAgentCallback {
    void idSet(in String id);
    String idGet();
    oneway void receiveEvent(in String[] tv,in byte[] data);
    oneway void onActivityResult(in int requestCode, in int resultCode, in Intent data);
}

