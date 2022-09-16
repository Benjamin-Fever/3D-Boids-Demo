
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"


using namespace glm;
using namespace std;


vec3 Boid::color() const {
    if (m_id == -1) return {1, 0, 0};
    else if (m_id == 0) return {0, 1, 0};
    else if (m_id == 1) return {0, 0, 1};
    return {0, 1, 0};
}

glm::vec3 Boid::cohesion(Scene *scene){
    if (this->m_id == -1) { return {0,0,0}; }
    glm::vec3 force(0, 0, 0);
    int count = 1;
    for (Boid const&b : scene->boids()){
        if (&b != this && distance(this->position(), b.position()) < scene->m_neighbor_radius && b.m_id == this->m_id) {
            force += b.position();
            count ++;
        }
    }
    force /= count;
    return force - this->position();
}

glm::vec3 Boid::alignment(Scene *scene){
    if (this->m_id == -1) {return {0, 0, 0}; }
    glm::vec3 force(0,0,0);
    int count = 1;
    for (Boid const&b : scene->boids()){
        if (&b != this && distance(this->position(), b.position()) < scene->m_neighbor_radius && b.m_id == this->m_id){
            force += b.velocity();
            count ++;
        }
    }
    force /= count;
    return force - this->velocity();
}

glm::vec3 Boid::avoidance(Scene *scene){
    glm::vec3 force(0.0f);
    for (Boid const&b : scene->boids()) {
        if (&b != this && distance(this->position(), b.position()) < scene->m_neighbor_radius) {
            vec3 displacment = this->position() - b.position();
            float distance = length(displacment);
            force += displacment / (distance * distance);

        }
    }
    return force;
}

void Boid::calculateForces(Scene *scene) {
    m_acceleration += (avoidance(scene) * scene->m_avoidance_weight + cohesion(scene) * scene->m_cohesion_weight + alignment(scene) * scene->m_alignment_weight )/1.0f;
    m_acceleration = clamp(m_acceleration, -scene->m_max_accaleration, scene->m_max_accaleration);
}

void Boid::update(float timestep, Scene *scene) {

    m_velocity += m_acceleration * timestep;
    if (this->m_id == -1){

    }
    else{

    }
    m_velocity = clamp(length(m_velocity), scene->m_min_speed, scene->m_max_speed) * normalize(velocity());
	m_position += m_velocity * timestep;

    if(m_position.x > scene->bound().x){m_position.x = -scene->bound().x;}
    else if(m_position.x < -scene->bound().x){m_position.x = scene->bound().x;}
    else if (m_position.y > scene->bound().y){m_position.y = -scene->bound().y;}
    else if(m_position.y < -scene->bound().y){m_position.y = scene->bound().y;}
    else if (m_position.z > scene->bound().z){m_position.z = -scene->bound().z;}
    else if(m_position.z < -scene->bound().z){m_position.z = scene->bound().z;}
    m_acceleration = {0,0,0};
}