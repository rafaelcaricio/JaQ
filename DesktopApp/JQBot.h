

#ifndef JQ_BOT_H
#define JQ_BOT_H

#include "JQMisc.h"
#include "JQLeg.h" 
#include "JQServo.h"

class JQLeg;
class JQServo;



class JQBot
{
public:
    JQBot();
    ~JQBot() = default;
    JQLeg& leg(JQLegID id) { return *pLeg[id]; }
    JQServo& servo(JQServoID id) { return *pServo[id]; }
protected:
private:
    JQLeg* pLeg[4];
    JQServo* pServo[12];
};


#endif
