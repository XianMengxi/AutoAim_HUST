#ifndef _ENEMY_TYPE_H
#define _ENEMY_TYPE_H
enum ENEMY_TYPE
{
    NOT_FOUND = -1,
    Undefined = 0,
    Hero = 1,
    Engineer = 2,
    Infantry1 = 3,
    Infantry2 = 4,
    Infantry3 = 5,
    Sentry = 6,
    Outpost = 7,
    Base = 8,
    NOT_BUFF = 9,
    BUFF = 10,
    BUFF5 = 11 //使用五点进行PNP计算
};
#endif