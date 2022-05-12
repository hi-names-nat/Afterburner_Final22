#pragma once
#include <WO.h>
#include "DetourCrowd.h"

namespace Aftr {
    class WODetourActor : public WO
    {

        WODetourActor();

        
        void onUpdateWO() override;

        void setDestination(Vector position);

        int agentIndex;
        dtCrowdAgentParams agentParams;

        dtCrowd* myCrowd;

    public:
        static WODetourActor* New(dtCrowd* crowd, Vector startPosition, dtCrowdAgentParams params);

    };
}
