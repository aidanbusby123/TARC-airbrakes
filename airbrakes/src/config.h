#include "main.h"

class config {
    private:
        float dragForceCoefCoefs[3];
        float deploymentTimeCoefs[3];    
        float refArea;
        float targetApogee;
    public:
        StaticJsonDocument<512> configJSON;
        SdFile configFile;
        
        int begin(const char* filename);
        void loadConfig();

        float *getDragForceCoefCoefs(){
            return dragForceCoefCoefs;
        }

        float getRefArea();
        float getTargetApogee();
};