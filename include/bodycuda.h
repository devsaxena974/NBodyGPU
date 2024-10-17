#ifndef __BODYCUDA_H__
#define __BODYCUDA_H__

#include "body.h"

class BodyCUDA : public Body {
    public:
        // constructors
        BodyCUDA(int numBodies);
        BodyCUDA(int numBodies, unsigned int p, unsigned int q);

        // deconstructor
        virtual ~BodyCUDA();

        // update states given a dt
        virtual void update(float deltaTime);

        // setters for softening, damping
        virtual void setSoftening(float softening);
        virtual void setDamping(float damping);

        // getter and setter for array of bodies
        virtual float* getArray(BodyArray array);
        virtual void setArray(BodyArray array, const float* data);

        // retrieve the current buffer
        virtual unsigned int getCurrentReadBuffer() const {
            return m_pbo[m_currentRead];
        }
    protected:
        BodyCUDA() {}

        // functions to init and finalize the body system
        virtual void _initialize(int numBodies);
        virtual void _finalize();
    protected:
        // variables on CPU memory
        float* m_hPos;
        float* m_hVel;

        // variables on GPU memory
        float* m_dPos[2];
        float* m_dVel[2];

        float m_damping;

        // pixel buffer object to efficiently transfer data between GPU and rendering
        unsigned int m_pbo[2];
        unsigned int m_currentRead;
        unsigned int m_currentWrite;

        // measure performance or runtime of CUDA kernel
        unsigned int m_timer;

        // CUDA grid and block
        unsigned int m_p;
        unsigned int m_q;
};

#endif