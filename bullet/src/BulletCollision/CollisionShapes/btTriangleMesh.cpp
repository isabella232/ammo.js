/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btTriangleMesh.h"



btTriangleMesh::btTriangleMesh (bool use32bitIndices,bool use4componentVertices)
:m_use32bitIndices(use32bitIndices),
m_use4componentVertices(use4componentVertices),
m_weldingThreshold(0.0)
{
	btIndexedMesh meshIndex;
	meshIndex.m_numTriangles = 0;
	meshIndex.m_numVertices = 0;
	meshIndex.m_indexType = PHY_INTEGER;
	meshIndex.m_triangleIndexBase = 0;
	meshIndex.m_triangleIndexStride = 3*sizeof(int);
	meshIndex.m_vertexBase = 0;
	meshIndex.m_vertexStride = sizeof(btVector3);
	m_indexedMeshes.push_back(meshIndex);

	if (m_use32bitIndices)
	{
		m_indexedMeshes[0].m_numTriangles = m_32bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = PHY_INTEGER;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(int);
	} else
	{
		m_indexedMeshes[0].m_numTriangles = m_16bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = PHY_SHORT;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(short int);
	}

	if (m_use4componentVertices)
	{
		m_indexedMeshes[0].m_numVertices = m_4componentVertices.size();
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = sizeof(btVector3);
	} else
	{
		m_indexedMeshes[0].m_numVertices = m_3componentVertices.size()/3;
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = 3*sizeof(btScalar);
	}


}

void	btTriangleMesh::addIndex(int index)
{
	if (m_use32bitIndices)
	{
		m_32bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_32bitIndices[0];
	} else
	{
		m_16bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_16bitIndices[0];
	}
}


int	btTriangleMesh::findOrAddVertex(const btVector3& vertex, bool removeDuplicateVertices)
{
	//return index of new/existing vertex
	///@todo: could use acceleration structure for this
	if (m_use4componentVertices)
	{
		if (removeDuplicateVertices)
			{
			for (int i=0;i< m_4componentVertices.size();i++)
			{
				if ((m_4componentVertices[i]-vertex).length2() <= m_weldingThreshold)
				{
					return i;
				}
			}
		}
		m_indexedMeshes[0].m_numVertices++;
		m_4componentVertices.push_back(vertex);
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_4componentVertices[0];

		return m_4componentVertices.size()-1;
		
	} else
	{
		
		if (removeDuplicateVertices)
		{
			for (int i=0;i< m_3componentVertices.size();i+=3)
			{
				btVector3 vtx(m_3componentVertices[i],m_3componentVertices[i+1],m_3componentVertices[i+2]);
				if ((vtx-vertex).length2() <= m_weldingThreshold)
				{
					return i/3;
				}
			}
		}
		m_3componentVertices.push_back(vertex.getX());
		m_3componentVertices.push_back(vertex.getY());
		m_3componentVertices.push_back(vertex.getZ());
		m_indexedMeshes[0].m_numVertices++;
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_3componentVertices[0];
		return (m_3componentVertices.size()/3)-1;
	}

}
		
void	btTriangleMesh::addTriangle(const btVector3& vertex0,const btVector3& vertex1,const btVector3& vertex2,bool removeDuplicateVertices)
{
	m_indexedMeshes[0].m_numTriangles++;
	addIndex(findOrAddVertex(vertex0,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex1,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex2,removeDuplicateVertices));
}

int btTriangleMesh::getNumTriangles() const
{
	if (m_use32bitIndices)
	{
		return m_32bitIndices.size() / 3;
	}
	return m_16bitIndices.size() / 3;
}

void btTriangleMesh::preallocateVertices(int numverts)
{
	if (m_use4componentVertices)
	{
		m_4componentVertices.reserve(numverts);
	} else
	{
		m_3componentVertices.reserve(numverts);
	}
}

void btTriangleMesh::preallocateIndices(int numindices)
{
	if (m_use32bitIndices)
	{
		m_32bitIndices.reserve(numindices);
	} else
	{
		m_16bitIndices.reserve(numindices);
	}
}

////////////////////////////////////////////
// Mackey Kinard - BT SMOOTH TRIANGLE MESH
////////////////////////////////////////////

btSmoothTriangleMesh::btSmoothTriangleMesh(): m_weldingThreshold(0.0), m_hasVertexNormals(false), m_useTriangleNormals(false)
{
	//Initialization for btTriangleVertexArray
	btIndexedMesh meshIndex;
	meshIndex.m_numTriangles = 0;
	meshIndex.m_numVertices = 0;
	meshIndex.m_indexType = PHY_INTEGER;
	meshIndex.m_triangleIndexBase = 0;
	meshIndex.m_triangleIndexStride = 3 * sizeof(int);
	meshIndex.m_vertexBase = 0;
	meshIndex.m_vertexStride = sizeof(btVector3);
	m_indexedMeshes.push_back(meshIndex);

	//Indices
	m_indexedMeshes[0].m_numTriangles = m_indices.size() / 3;
	m_indexedMeshes[0].m_triangleIndexBase = 0;
	m_indexedMeshes[0].m_indexType = PHY_INTEGER;
	m_indexedMeshes[0].m_triangleIndexStride = 3 * sizeof(int);

	//Vertices
	m_indexedMeshes[0].m_numVertices = m_vertices.size() / 3;
	m_indexedMeshes[0].m_vertexBase = 0;
	m_indexedMeshes[0].m_vertexStride = 3 * sizeof(btScalar);
}

void btSmoothTriangleMesh::addTriangle(const btVector3& vertex0, const btVector3& vertex1, const btVector3& vertex2, bool removeDuplicateVertices)
{
	m_indexedMeshes[0].m_numTriangles++;
	int v0 = findOrAddVertex(vertex0, removeDuplicateVertices);
	int v1 = findOrAddVertex(vertex1, removeDuplicateVertices);
	int v2 = findOrAddVertex(vertex2, removeDuplicateVertices);

	addIndex(v0);
	addIndex(v1);
	addIndex(v2);
}

void btSmoothTriangleMesh::addTriangleNormals(const btVector3& vertex0, const btVector3& vertex1, const btVector3& vertex2, const btVector3& normal0, const btVector3& normal1, const btVector3& normal2, bool removeDuplicateVertices)
{
	m_indexedMeshes[0].m_numTriangles++;
	int v0 = findOrAddVertex(vertex0, removeDuplicateVertices);
	int v1 = findOrAddVertex(vertex1, removeDuplicateVertices);
	int v2 = findOrAddVertex(vertex2, removeDuplicateVertices);

	addIndex(v0);
	addIndex(v1);
	addIndex(v2);

	m_normals.push_back(normal0);
	m_normals.push_back(normal1);
	m_normals.push_back(normal2);
	m_hasVertexNormals = true;
}

void btSmoothTriangleMesh::preallocateVertices(int numverts)
{
	m_vertices.reserve(numverts);
	m_normals.reserve(numverts);
}

void btSmoothTriangleMesh::preallocateIndices(int numindices)
{
	m_indices.reserve(numindices);
}

int btSmoothTriangleMesh::findOrAddVertex(const btVector3& vertex, bool removeDuplicateVertices)
{
	if (removeDuplicateVertices)
	{
		for (int i = 0; i < m_vertices.size(); i += 3)
		{
			btVector3 vtx(m_vertices[i], m_vertices[i + 1], m_vertices[i + 2]);
			if ((vtx - vertex).length2() <= m_weldingThreshold)
			{
				return i / 3;
			}
		}
	}
	m_vertices.push_back(vertex.getX());
	m_vertices.push_back(vertex.getY());
	m_vertices.push_back(vertex.getZ());

	m_indexedMeshes[0].m_numVertices++;
	m_indexedMeshes[0].m_vertexBase = (unsigned char*)& m_vertices[0];
	return (m_vertices.size() / 3) - 1;
}

void btSmoothTriangleMesh::addIndex(int index)
{
	m_indices.push_back(index);
	m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*)& m_indices[0];
}

btVector3& btSmoothTriangleMesh::getVertexNormal(int vertexIndex)
{
	return m_normals[vertexIndex];
}

// shape, subpart and triangle come from the ray callback.
// transform is the mesh shape's world transform
// position is the world space hit point of the ray
btVector3 btSmoothTriangleMesh::getTriangleMeshNormal(const btTransform &transform, btStridingMeshInterface *mesh_interface, int subpart, int triangle)
{
	const unsigned char *vertexbase;
	int num_verts;
	PHY_ScalarType type;
	int stride;

	const unsigned char *indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;

	int numverts = 0;
	mesh_interface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, subpart);

	const unsigned int *indices = (const unsigned int *)(indexbase + triangle * indexstride);
	unsigned int i = indices[0], j = indices[1], k = indices[2];
	// ..
	btVector3 n1 = this->getVertexNormal(i);
	btVector3 n2 = this->getVertexNormal(j);
	btVector3 n3 = this->getVertexNormal(k);
	//printf("GetTriangleMeshNormal(): Mesh Vertex Normals I: (%f x %f x %f) -> J: (%f x %f x %f) -> K: (%f x %f x %f)\r\n", n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z(), n3.x(), n3.y(), n3.z());
	btVector3 result = this->getVertexNormal(i) + this->getVertexNormal(j) + this->getVertexNormal(k);
	// ..
	// Transform back into world space
	// ..
	result = transform.getBasis() * result;
	result.normalize();
	mesh_interface->unLockReadOnlyVertexBase(subpart);
	return result;
}

// shape, subpart and triangle come from the ray callback.
// transform is the mesh shape's world transform
// position is the world space hit point of the ray
btVector3 btSmoothTriangleMesh::interpolateMeshNormal(const btTransform &transform, btStridingMeshInterface *mesh_interface, int subpart, int triangle, const btVector3 &position)
{
	const unsigned char *vertexbase;
	int num_verts;
	PHY_ScalarType type;
	int stride;

	const unsigned char *indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;

	int numverts = 0;
	mesh_interface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, subpart);

	// Calculate new barycentric coordinates
	const unsigned int *indices = (const unsigned int *)(indexbase + triangle * indexstride);
	unsigned int i = indices[0], j = indices[1], k = indices[2];
	StrideVertexAccessor positions(vertexbase, stride, 0);
	btVector3 barry = this->barycentricCoordinates(transform.invXform(position), positions[i], positions[j], positions[k]);
	// ..
	btVector3 n1 = this->getVertexNormal(i);
	btVector3 n2 = this->getVertexNormal(j);
	btVector3 n3 = this->getVertexNormal(k);
	//printf("InterpolateMeshNormal(): Mesh Vertex Normals I: (%f x %f x %f) -> J: (%f x %f x %f) -> K: (%f x %f x %f)\r\n", n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z(), n3.x(), n3.y(), n3.z());
	// ..
	// Interpolate from barycentric coordinates
	// ..
	btVector3 result = barry.x() * this->getVertexNormal(i) + barry.y() * this->getVertexNormal(j) + barry.z() * this->getVertexNormal(k);
	// ..
	// Transform back into world space
	// ..
	result = transform.getBasis() * result;
	result.normalize();
	mesh_interface->unLockReadOnlyVertexBase(subpart);
	return result;
}

btVector3 btSmoothTriangleMesh::barycentricCoordinates(const btVector3 &position, const btVector3 &p1, const btVector3 &p2, const btVector3 &p3)
{
	btVector3 edge1 = p2 - p1;
	btVector3 edge2 = p3 - p1;
	// Area of triangle ABC
	btScalar p1p2p3 = edge1.cross(edge2).length2();
	// Area of BCP
	btScalar p2p3p = (p3 - p2).cross(position - p2).length2(); // Area of CAP
	btScalar p3p1p = edge2.cross(position - p3).length2();
	btScalar s = btSqrt(p2p3p / p1p2p3);
	btScalar t = btSqrt(p3p1p / p1p2p3);
	btScalar w = 1.0f - s - t;
	return btVector3(s, t, w);
}
