#ifndef __BODY_H__
#define __BODY_H__

// Create a class for the body system that can
//  be extended to GPU and CPU classes

enum NBodyConfig {
    NBODY_CONFIG_RANDOM,
    NBODY_CONFIG_EXPAND
};

// utility function to randomly initialize the bodies
void randomizeBodies(NBodyConfig cfg, float* pos, float* vel, float* color, float clusterScale, float velocityScale, int numBodies);

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
        // set damping factor on the velocities to simulate energy loss
        //  or to stabilize the simulation
        virtual void setDamping(float damping) = 0;

        // get a pointer to either position or velocities array of the body system
        virtual float* getArray(BodyArray array) = 0;
        virtual void setArray(BodyArray array, const float* data) = 0;

        // for gpu implementation, returns a device buffer ID
        virtual unsigned int getCurrentReadBuffer() const = 0;

        // return num bodies in the simulation
        int getNumBodies() const;

    protected:
        // number of bodies in the system
        int m_numBodies;

        // flag indicating whether or not system is initialized
        bool m_bInitialized;

        // default constructor
        Body() {}

        // methods to handle init of resources, memory alloc, etc
        //  based on num bodies
        virtual void _intiialize(int numBodies) = 0;
    
        // similar method to clean up resources
        virtual void _finalize() = 0;

};

#endif