#ifndef __BODYCPU_H__
#define __BODYCPU_H__

#include "body.h"

class BodyCPU : public Body {
    public:
        BodyCPU(int numBodies);
        virtual ~BodyCPU();
        
        virtual void update(float deltaTIme);

        virtual void setSoftening(float softening) { 
            m_softeningSquared = softening * softening;
        }
        virtual void setDamping(float damping) { 
            m_damping = damping;
        }

        virtual float* getArray(BodyArray array);
        virtual void setArray(BodyArray array, const float* data);

        virtual unsigned int getCurrentReadBuffer() const {
            return m_currentRead;
        }

    protected:
        // methods
        BodyCPU() {}
        virtual void _initialize(int numBodies);
        virtual void _finalize();

        void _computeNBodyGravitation();
        void _integrateNBodySystem(float deltaTime);

        // data
        float* m_pos[2];
        float* m_vel[2];
        float* m_force;

        float m_softeningSquared;
        float m_damping;

        unsigned int m_currentRead;
        unsigned int m_currentWrite;

        unsigned int m_timer;
};

#endif