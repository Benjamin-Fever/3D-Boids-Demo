
#pragma once

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"


class Boid {
private:

	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;

    int m_id;

public:
	Boid(glm::vec3 pos, glm::vec3 dir) : m_position(pos), m_velocity(dir), m_id(0) {}
    Boid(glm::vec3 pos, glm::vec3 dir, int id) : m_position(pos), m_velocity(dir), m_id(id) {}

	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;

	void calculateForces(Scene *scene);
	void update(float timestep, Scene *scene);

    glm::vec3 cohesion(Scene *scene);
    glm::vec3 alignment(Scene *scene);
    glm::vec3 avoidance(Scene *scene);
};