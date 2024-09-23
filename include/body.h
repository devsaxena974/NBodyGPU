#ifndef __BODY_H__
#define __BODY_H__

// Create a class for the body system that can
//  be extended to GPU and CPU classes

class Body {
    public:
        // constructor
        Body(int numBodies) : m_numBodies(numBodies), m_bInitialized(false) {}

        // pure virtual update method to enforce derivation from child classes
        //  takes in a time step
        virtual void update(float deltaTime) = 0;

        // define an enumerator with the body positions and velocities
        enum BodyArray {
            BODYSYSTEM_POSITION,
            BODYSYSTEM_VELOCITY
        };

        // set gravitational softening parameter
        //  to prevent numerical errors when bodies close to each other
        virtual void setSoftening(float softening) = 0;

    protected:
        // number of bodies in the system
        int m_numBodies;

        // flag indicating whether or not system is initialized
        bool m_bInitialized;
};

#endif