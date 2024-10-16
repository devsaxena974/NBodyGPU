#include "bodycpu.h"

#include <assert.h>
#include <memory.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
// I think cutil doesn't exist anymore????
//#include <cutil.h>
#include<algorithm>

// Constructor
BodyCPU::BodyCPU(int numBodies):
    Body(numBodies),
    m_force(0),
    m_softeningSquared(.001f),
    m_damping(0.995f),
    m_currentRead(0),
    m_currentWrite(0),
    m_timer(0)
{

    // first two timesteps of pos and vel should be initialized to 0
    m_pos[0] = 0;
    m_pos[1] = 0;
    m_vel[0] = 0;
    m_vel[1] = 0;

    _initialize(numBodies);
}

// Destructor
BodyCPU::~BodyCPU() {
    // clean up allocated resources
    _finalize();

    m_numBodies = 0;
}

// Initialization
void BodyCPU::_initialize(int numBodies) {
    // make sure system is not already initialized
    assert(!m_bInitialized);

    m_numBodies = numBodies;

    // allocate memory for pos, vel, and forces
    m_pos[0] = new float[m_numBodies * 4];
    m_pos[1] = new float[m_numBodies * 4];
    m_vel[0] = new float[m_numBodies * 4];
    m_vel[1] = new float[m_numBodies * 4];
    m_force = new float[m_numBodies * 4];

    // initialize all allocated memory to 0
    memset(m_pos[0], 0, m_numBodies * sizeof(float) * 4);
    memset(m_pos[1], 0, m_numBodies * sizeof(float) * 4);
    memset(m_vel[0], 0, m_numBodies * sizeof(float) * 4);
    memset(m_vel[1], 0, m_numBodies * sizeof(float) * 4);
    memset(m_force, 0, m_numBodies * sizeof(float) * 4);

    // set initialized flag to true
    m_bInitialized = true;
}

// Finalization
void BodyCPU::_finalize() {
    // Make sure system was initialized
    assert(m_bInitialized);

    // Release memory that was allocated for pos, vel, force arrays
    delete[] m_pos[0];
    delete[] m_pos[1];
    delete[] m_vel[0];
    delete[] m_vel[1];
    delete[] m_force;
}

// Update Timestep
void BodyCPU::update(float deltaTime) {
    assert(m_bInitialized);

    // Call integrate function to integrate all n bodies of the system
    _integrateNBodySystem(deltaTime);

    // Swap the current read and write buffers so that we can read from one buffer
    //  and write to the next
    std::swap(m_currentRead, m_currentWrite);
}

// Get Either Pos or Vel Array
float* BodyCPU::getArray(BodyArray array) {
    assert(m_bInitialized);

    float* data = nullptr;

    // Return pos or vel array based on which one is requested
    switch(array) {
        default:
        case BODYSYSTEM_POSITION:
            // set data to current position array accessing correct one with buffer variable
            data = m_pos[m_currentRead];
            break;
        case BODYSYSTEM_VELOCITY:
            // set data to current position array accessing correct one with buffer variable
            data = m_vel[m_currentRead];
            break;
    }

    return data;
}

// Set Either Pos or Vel Array
void BodyCPU::setArray(BodyArray array, const float* data) {
    assert(m_bInitialized);

    float* target = nullptr;

    // Figure out which array to access
    switch(array) {
        default:
        case BODYSYSTEM_POSITION:
            // set data to current position array accessing correct one with buffer variable
            target = m_pos[m_currentRead];
            break;
        case BODYSYSTEM_VELOCITY:
            // set data to current position array accessing correct one with buffer variable
            target = m_vel[m_currentRead];
            break;
    }

    // Copy the data into the correct array
    memcpy(target, data, m_numBodies * sizeof(float) * 4);
}

// Set tolerance variables
const float TOLERANCE = 3e-7f;
const float FUDGE_FACTOR = 0.025f;

// define normalize function
extern void normalize(float vector[3]);
// define func for body-body interaction for gravitational calculation
extern void bodyBodyInteraction(float accel[3], float posMass0[4], float posMass1[4], float softeningSquared);

void BodyCPU::_computeNBodyGravitation() {
    for(int i = 0; i < m_numBodies; ++i) {
        // initialize all forces to 0 for each body i
        m_force[i*4] = 0;
        m_force[i*4 + 1] = 0;
        m_force[i*4 + 2] = 0;

        for(int j = 0; j < m_numBodies; ++j) {
            // compute gravitational acceleration for every OTHER body j
            float accel[3] = {0, 0, 0};
            bodyBodyInteraction(accel, &m_pos[m_currentRead][i*4], &m_pos[m_currentRead][j*4], m_softeningSquared);
            for(int k = 0; k < 3; ++k) {
                m_force[i * 4 + k] += accel[k];
            }
        }
    }
}

// Now we integrate!!
// The numerical integration scheme we use is the semi-implicit Euler method
void BodyCPU::_integrateNBodySystem(float deltaTime) {
    // first compute the gravitation acceleration for each of the bodies
    _computeNBodyGravitation();

    for(int i = 0; i < m_numBodies; ++i) {
        // create index offset variable
        int idx = 4 * i;
        // create arrays to hold pos, vel, force values for cur body
        float pos[3];
        float vel[3];
        float force[3];
        // fill pos array with values in the current buffer
        pos[0] = m_pos[m_currentRead][idx+0];
        pos[1] = m_pos[m_currentRead][idx+1];
        pos[2] = m_pos[m_currentRead][idx+2];
        float mass = m_pos[m_currentRead][idx+3];

        vel[0] = m_vel[m_currentRead][idx+0];
        vel[1] = m_vel[m_currentRead][idx+1];
        vel[2] = m_vel[m_currentRead][idx+2];
        float invMass = m_vel[m_currentRead][idx+3];

        force[0] = m_force[idx+0];
        force[1] = m_force[idx+1];
        force[2] = m_force[idx+2];

        // a = f/m
        // v_1 = v_0 + a * dt

        vel[0] += (force[0] * invMass) * deltaTime;
        vel[1] += (force[1] * invMass) * deltaTime;
        vel[2] += (force[2] * invMass) * deltaTime;

        // gotta damp the velocities by the damping factor
        vel[0] *= m_damping;
        vel[1] *= m_damping;
        vel[2] *= m_damping;

        // x_1 = x_0 + v * dt

        pos[0] += vel[0] * deltaTime;
        pos[1] += vel[1] * deltaTime;
        pos[2] += vel[2] * deltaTime;

        // update current buffer with integrated values
        m_pos[m_currentRead][idx+0] = pos[0];
        m_pos[m_currentRead][idx+1] = pos[1];
        m_pos[m_currentRead][idx+2] = pos[2];
        m_pos[m_currentRead][idx+3] = mass;

        m_vel[m_currentRead][idx+0] = vel[0];
        m_vel[m_currentRead][idx+1] = vel[1];
        m_vel[m_currentRead][idx+2] = vel[2];
        m_vel[m_currentRead][idx+3] = invMass;
    }
}

// utility vector operations
struct float3 {
    float x;
    float y;
    float z;
};

// method to scale a vector by a given scalar
float3 scale(float3& vec, float scalar) {
    float3 newVec = vec;
    newVec.x *= scalar;
    newVec.y *= scalar;
    newVec.z *= scalar;
    return newVec;
}

// method to normalize a vector
float normalize(float3& vec) {
    float distance = sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if(distance < 1e-6) {
        vec.x /= distance;
        vec.y /= distance;
        vec.z /= distance;
    }
    return distance;
}

// method to find the dot product of 2 vectors
float dot(float3 u, float3 v) {
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

// method to find the cross product of two vectors
float3 cross(float3 u, float3 v) {
    float3 newVec;
    newVec.x = u.y*v.z - u.z*v.y;
    newVec.y = u.z*v.x - u.x*v.z;
    newVec.z = u.x*v.y - u.y*v.x;
    return newVec;
}

// utility function to randomly place the bodies
// taken straight from the GPUGems code
void randomizeBodies(NBodyConfig cfg, float* pos, float* vel, float* color, float clusterScale, float velocityScale, int  numBodies) {
    switch(cfg) {
        default:
        case NBODY_CONFIG_RANDOM: {
            float scale = clusterScale * std::max(1.0f, numBodies / (1024.f));
            float vscale = velocityScale * scale;

            // initialize indices for pos, vel, and body
            int p = 0;
            int v = 0;
            int i = 0;
            while(i < numBodies) {
                float3 point;

                point.x = rand() / (float) RAND_MAX * 2 - 1;
                point.y = rand() / (float) RAND_MAX * 2 - 1;
                point.z = rand() / (float) RAND_MAX * 2 - 1;
                float lenSquared = dot(point, point);
                if(lenSquared > 1) {
                    continue;
                }

                float3 velocity;
                velocity.x = rand() / (float) RAND_MAX * 2 - 1;
				velocity.y = rand() / (float) RAND_MAX * 2 - 1;
				velocity.z = rand() / (float) RAND_MAX * 2 - 1;
                lenSquared = dot(velocity, velocity);
                if (lenSquared > 1) {
                    continue;
                }

                pos[p++] = point.x * scale;
                pos[p++] = point.y * scale;
                pos[p++] = point.z * scale;
                pos[p++] = 1.0f;

                vel[v++] = velocity.x * vscale;
                vel[v++] = velocity.y * scale;
                vel[v++] = velocity.z * scale;
                vel[v++] = 1.0f;

                i++;
            }
        }
        break;
        case NBODY_CONFIG_EXPAND: {
            float scale = clusterScale * std::max(1.0f, numBodies / (1024.f));
            float vscale = scale * velocityScale;

            int p = 0;
            int v = 0;
            for(int i = 0; i < numBodies;) {
                float3 point;

                point.x = rand() / (float) RAND_MAX * 2 - 1;
                point.y = rand() / (float) RAND_MAX * 2 - 1;
                point.z = rand() / (float) RAND_MAX * 2 - 1;
                float lenSquared = dot(point, point);
                if(lenSquared > 1) {
                    continue;
                }

                pos[p++] = point.x * scale; // pos.x
				pos[p++] = point.y * scale; // pos.y
				pos[p++] = point.z * scale; // pos.z
				pos[p++] = 1.0f; // mass
				vel[v++] = point.x * vscale; // pos.x
				vel[v++] = point.y * vscale; // pos.x
				vel[v++] = point.z * vscale; // pos.x
				vel[v++] = 1.0f; // inverse mass

                i++;
            }
        }
        break;

    }

    if(color) {
        int v = 0;
        for(int i = 0; i < numBodies; i++) {
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = 1.0f;
        }
    }
}

