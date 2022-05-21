#pragma once
#include <WO.h>
#include "DetourCrowd.h"

namespace Aftr {
    class WODetourActor : public WO
    {

        WODetourActor();

        
        void onUpdateWO() override;

        dtCrowd* myCrowd;

        float radius = .5f, maxSpeed = 3.5f, maxAcceleration = 8.0f, height = 2.0f;
        unsigned char updateFlags = 0;

    public:
        void setDestination(Vector position);


        static WODetourActor* New(dtCrowd* crowd, Vector startPosition);
        int agentIndex;


    };
}
