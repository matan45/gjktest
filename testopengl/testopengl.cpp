#include <iostream>
#include <glm.hpp>
#include <gtx/string_cast.hpp>
#include <vector>
#include <array>

using namespace glm;

#define GJK_MAX_NUM_ITERATIONS 64

struct Collider {
	virtual vec3 FindFurthestPoint(vec3 direction) const = 0;
};

struct MeshCollider : Collider
{
public:
	std::vector<vec3> m_vertices;

public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		vec3 maxPoint = vec3(0, 0, 0);
		float maxDistance = -FLT_MAX;

		for (const vec3& vertex : m_vertices) {
			float distance = dot(vertex, direction);
			if (distance > maxDistance) {
				maxDistance = distance;
				maxPoint = vertex;
			}
		}

		return maxPoint;
	}
};

struct SphereCollider : Collider
{
public:
	vec3 center;
	float radius;

public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		return direction * radius + center;
	}
};

struct BoxCollider : Collider
{
public:
	vec3 min, max;

public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		vec3 result;
		result.x = (direction.x > 0) ? max.x : min.x;
		result.y = (direction.y > 0) ? max.y : min.y;
		result.z = (direction.z > 0) ? max.z : min.z;
		return result;
	}
};

struct LineCollider : Collider
{
public:
	vec3 start, end;

public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		if (glm::dot(start, direction) > glm::dot(end, direction))
			return start;
		return end;
	}
};

struct CapsuleCollider : Collider
{
public:
	float r, y_base, y_cap;
	vec3 position;
public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		vec3 result = direction * r;
		result.y += (direction.y > 0) ? y_cap : y_base;
		return result+position;
	}
};

struct CylinderCollider : Collider
{
public:
	float r, y_base, y_cap;
	vec3 position;
public:
	vec3 FindFurthestPoint(vec3 direction) const override
	{
		vec3 dir_xz = vec3(direction.x, 0, direction.z);
		vec3 result = dir_xz * r;
		result.y += (direction.y > 0) ? y_cap : y_base;
		return result+position;
	}
};


struct Simplex {
private:
	std::array<vec3, 4> m_points;
	unsigned m_size = 0;

public:
	Simplex()
		: m_points({ vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0) })
		, m_size(0)
	{}

	Simplex& operator=(std::initializer_list<vec3> list) {
		for (auto v = list.begin(); v != list.end(); v++) {
			m_points[std::distance(list.begin(), v)] = *v;
		}
		m_size = list.size();

		return *this;
	}

	void push_front(vec3 point) {
		m_points = { point, m_points[0], m_points[1], m_points[2] };
		m_size = std::min(m_size + 1, 4u);
	}

	vec3& operator[](unsigned i) { return m_points[i]; }
	unsigned size() const { return m_size; }

	auto begin() const { return m_points.begin(); }
	auto end()   const { return m_points.end() - (4 - m_size); }
};

bool SameDirection(const vec3& direction, const vec3& ao)
{
	return dot(direction, ao) > 0;
}

bool Line(Simplex& points, vec3& direction)
{
	vec3 a = points[0];
	vec3 b = points[1];

	vec3 ab = b - a;
	vec3 ao = -a;

	if (SameDirection(ab, ao)) {
		direction = cross(cross(ab, ao), ab);
	}

	else {
		points = { a };
		direction = ao;
	}

	return false;
}

bool Triangle(Simplex& points, vec3& direction)
{
	vec3 a = points[0];
	vec3 b = points[1];
	vec3 c = points[2];

	vec3 ab = b - a;
	vec3 ac = c - a;
	vec3 ao = -a;

	vec3 abc = cross(ab, ac);

	if (SameDirection(cross(abc, ac), ao)) {
		if (SameDirection(ac, ao)) {
			points = { a, c };
			direction = cross(cross(ac, ao), ac);
		}

		else {
			return Line(points = { a, b }, direction);
		}
	}

	else {
		if (SameDirection(cross(ab, abc), ao)) {
			return Line(points = { a, b }, direction);
		}

		else {
			if (SameDirection(abc, ao)) {
				direction = abc;
			}

			else {
				points = { a, c, b };
				direction = -abc;
			}
		}
	}

	return false;
}

bool Tetrahedron(Simplex& points, vec3& direction)
{
	vec3 a = points[0];
	vec3 b = points[1];
	vec3 c = points[2];
	vec3 d = points[3];

	vec3 ab = b - a;
	vec3 ac = c - a;
	vec3 ad = d - a;
	vec3 ao = -a;

	vec3 abc = cross(ab, ac);
	vec3 acd = cross(ac, ad);
	vec3 adb = cross(ad, ab);

	if (SameDirection(abc, ao)) {
		return Triangle(points = { a, b, c }, direction);
	}

	if (SameDirection(acd, ao)) {
		return Triangle(points = { a, c, d }, direction);
	}

	if (SameDirection(adb, ao)) {
		return Triangle(points = { a, d, b }, direction);
	}

	return true;
}

bool NextSimplex(Simplex& points, vec3& direction)
{
	switch (points.size()) {
	case 2: return Line(points, direction);
	case 3: return Triangle(points, direction);
	case 4: return Tetrahedron(points, direction);
	}

	// never should be here
	return false;
}

vec3 Support(const Collider* colliderA, const Collider* colliderB, vec3 direction)
{
	return colliderA->FindFurthestPoint(direction) - colliderB->FindFurthestPoint(-direction);
}

bool GJK(const Collider* colliderA, const Collider* colliderB, Simplex& points) {
	// Get initial support point in any direction
	vec3 support = Support(colliderA, colliderB, vec3(1, 0, 0));

	// Simplex is an array of points, max count is 4
	//Simplex points;
	points.push_front(support);

	// New direction is towards the origin
	vec3 direction = -support;

	for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; iterations++) {
		support = Support(colliderA, colliderB, direction);

		if (dot(support, direction) <= 0) {
			return false; // no collision
		}

		points.push_front(support);
		if (NextSimplex(points, direction)) {
			return true;
		}
	}

	return false; // no collision
}

/// <summary>
///EPA
/// </summary>
struct CollisionPoints
{
	vec3 Normal;
	float PenetrationDepth;
	bool HasCollision;
};

void AddIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& faces, size_t a, size_t b)
{
	auto reverse = std::find(               //      0--<--3
		edges.begin(),                     //     / \ B /   A: 2-0
		edges.end(),                       //    / A \ /    B: 0-2
		std::make_pair(faces[b], faces[a]) //   1-->--2
	);

	if (reverse != edges.end()) {
		edges.erase(reverse);
	}

	else {
		edges.emplace_back(faces[a], faces[b]);
	}
}

std::pair<std::vector<vec4>, size_t> GetFaceNormals(
	const std::vector<vec3>& polytope,
	const std::vector<size_t>& faces)
{
	std::vector<vec4> normals;
	size_t minTriangle = 0;
	float  minDistance = FLT_MAX;

	for (size_t i = 0; i < faces.size(); i += 3) {
		vec3 a = polytope[faces[i]];
		vec3 b = polytope[faces[i + 1]];
		vec3 c = polytope[faces[i + 2]];

		vec3 normal = glm::normalize(glm::cross(b - a, c - a));
		float distance = glm::dot(normal, a);

		if (distance < 0) {
			normal *= -1;
			distance *= -1;
		}

		normals.emplace_back(normal, distance);

		if (distance < minDistance) {
			minTriangle = i / 3;
			minDistance = distance;
		}
	}

	return { normals, minTriangle };
}

CollisionPoints EPA(const Simplex& simplex, const Collider* colliderA, const Collider* colliderB) {
	std::vector<vec3> polytope(simplex.begin(), simplex.end());
	std::vector<size_t>  faces = {
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2
	};

	// list: vector4(normal, distance), index: min distance
	auto [normals, minFace] = GetFaceNormals(polytope, faces);

	vec3 minNormal;
	float   minDistance = FLT_MAX;
	while (minDistance == FLT_MAX) {
		//std::cout << minDistance << std::endl;
		minNormal.x = normals[minFace].x;
		minNormal.y = normals[minFace].y;
		minNormal.z = normals[minFace].z;

		minDistance = normals[minFace].w;

		vec3 support = Support(colliderA, colliderB, minNormal);
		float sDistance = glm::dot(minNormal, support);

		if (abs(sDistance - minDistance) > 0.001f) {
			minDistance = FLT_MAX;

			std::vector<std::pair<size_t, size_t>> uniqueEdges;

			for (size_t i = 0; i < normals.size(); i++) {
				if (SameDirection(normals[i], support)) {
					size_t f = i * 3;

					AddIfUniqueEdge(uniqueEdges, faces, f, f + 1);
					AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
					AddIfUniqueEdge(uniqueEdges, faces, f + 2, f);

					faces[f + 2] = faces.back(); faces.pop_back();
					faces[f + 1] = faces.back(); faces.pop_back();
					faces[f] = faces.back(); faces.pop_back();

					normals[i] = normals.back(); normals.pop_back();

					i--;
				}
			}

			std::vector<size_t> newFaces;
			for (auto& [edgeIndex1, edgeIndex2] : uniqueEdges) {
				newFaces.push_back(edgeIndex1);
				newFaces.push_back(edgeIndex2);
				newFaces.push_back(polytope.size());
			}

			polytope.push_back(support);

			auto [newNormals, newMinFace] = GetFaceNormals(polytope, newFaces);

			float oldMinDistance = FLT_MAX;
			for (size_t i = 0; i < normals.size(); i++) {
				if (normals[i].w < oldMinDistance) {
					oldMinDistance = normals[i].w;
					minFace = i;
				}
			}

			if (newNormals[newMinFace].w < oldMinDistance) {
				minFace = newMinFace + normals.size();
			}

			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.insert(normals.end(), newNormals.begin(), newNormals.end());
		}

	}

	if (minDistance == FLT_MAX) {
		return {};
	}

	CollisionPoints points;

	points.Normal = -minNormal;
	points.PenetrationDepth = minDistance + 0.001f;
	points.HasCollision = true;

	return points;
}

int main()
{
	SphereCollider sp1, sp2;
	sp1.center = vec3(5, 5, 0);
	sp1.radius = 1.0f;

	sp2.center = vec3(2, 2, 0);
	sp2.radius = 1.0f;
	Simplex simplex;

	if (GJK(&sp1, &sp2, simplex)) {
		std::cout << "hit" << std::endl;
		CollisionPoints points = EPA(simplex, &sp1, &sp2);
		std::cout << glm::to_string(simplex[0])<< glm::to_string(simplex[0]) << glm::to_string(simplex[0]) << glm::to_string(simplex[0]) << std::endl;
		std::cout << points.HasCollision << std::endl << points.PenetrationDepth << std::endl;
	}
	else {
		std::cout << "NO hit" << std::endl;
		
	}
	/**
	MeshCollider mm1, mm2;
	mm1.m_vertices.push_back(vec3(0, 0, 0));
	mm1.m_vertices.push_back(vec3(2, 2, 0));
	mm1.m_vertices.push_back(vec3(3, 3, 0));

	mm2.m_vertices.push_back(vec3(1, 1, 0));
	mm2.m_vertices.push_back(vec3(5, 5, 0));
	mm2.m_vertices.push_back(vec3(2, 2, 0));
	mm2.m_vertices.push_back(vec3(3, 3, 0));

	BoxCollider box;
	box.max = vec3(2, 2, 2);
	box.min = vec3(1, 1, 1);
	Simplex simplex2;
	Simplex simplex3;

	if (GJK(&mm1, &mm2, simplex2)) {
		std::cout << "hit MM" << std::endl;
		CollisionPoints points = EPA(simplex2, &mm1, &mm2);
		std::cout << simplex2.size() << std::endl;
		std::cout << points.HasCollision << std::endl << points.PenetrationDepth << std::endl;
	}
	else {
		std::cout << "NO hit MM" << std::endl;
	}

	if (GJK(&sp2, &box, simplex3)) {
		std::cout << "hit MP" << std::endl;
		CollisionPoints points = EPA(simplex3, &sp2, &mm2);
		std::cout << simplex3.size() << std::endl;
		std::cout << points.HasCollision << std::endl << points.PenetrationDepth << std::endl;
	}
	else {
		std::cout << "NO hit MP" << std::endl;
	}*/
}

