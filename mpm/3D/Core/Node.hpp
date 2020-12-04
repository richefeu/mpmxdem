#ifndef NODE_HPP_7BE4537F
#define NODE_HPP_7BE4537F

#include <vec3.hpp>
#include <mat9.hpp>

struct node
{
	vec3r pos;      // Position
	vec3r q;        // Momentum
	vec3r qdot;     // Derivative of Momentum
	vec3r f;        // Force
	vec3r fb;       // Interfacial force
	double mass;    // Mapped Mass
	bool xfixed;    // Null-velocity boolean along x
	bool yfixed;    // Null-velocity boolean along y
	bool zfixed;    // Null-velocity boolean along z
	mat9r stress;   // Node-stress (used e.g. for smoothing)
	vec3r vel;      // Node-velocity (used e.g. for smoothing)
	vec3r fnormal;
	vec3r ftang;
	double friction;
	double normalStiffness;
	double tangentialStiffness;
	vec3r velnormal;
	vec3r veltang;
	bool normalFixed;
	bool tangFixed;

    int number;

    // operator used to compare. This allows me to use std::sort in a vector made of nodes
    bool operator < (const node &other) const {  
        return number < other.number;
    }

    node();         // Ctor
};

#endif /* end of include guard: NODE_HPP_7BE4537F */
