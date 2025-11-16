package com.nuwarobotics.service.custombehavior;

// Declare any non-default types here with import statements

interface CustomBehavior {
    void onWelcome(String name, long faceid);
    void prepare(String parameter);
    void process(String parameter);
    void finish(String parameter);
}
