package com.nuwarobotics.service;
import com.nuwarobotics.service.agent.IKiwiAgentCallback;
import com.nuwarobotics.service.IClientId;

interface IKiwiService {
	oneway void cmd(in String[] tv, in byte[] data, in IClientId client, in int priority, in IKiwiAgentCallback cb);
	oneway void startActivity(in IClientId client, in Intent intent);
    boolean registerCallback(in IKiwiAgentCallback cb, in IClientId id);   
    boolean unregisterCallback(in IKiwiAgentCallback cb, in IClientId id);
    String cmd2(in String[] tv, in byte[] data, in IClientId client, in int priority, in IKiwiAgentCallback cb);
}
//
