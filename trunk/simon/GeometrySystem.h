/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef GEOMETRY_SYSTEM_H
#define GEOMETRY_SYSTEM_H

#include <simon/SmartPointer.h>
#include <simon/Id.h>
#include <simon/RigidBody.h>
#include <simon/Geometry.h>
#include <simon/Plane.h>
#include <simon/Sphere.h>
#include <simon/Box.h>
#include <simon/Capsule.h>
#include <simon/Vector3.h>
#include <simon/Interference.h>
#include <simon/config.h>
#include <vector>

#include <simon/CollisionDetection.h>

#define CollisionMessages SimonState::exemplar()->messages << "Collision:"

#ifdef _MSC_VER
#pragma warning( push, 1 )
#endif

#  include <boost/graph/strong_components.hpp>
#  include <boost/graph/graph_utility.hpp>
#  include <boost/graph/graphviz.hpp>


#ifdef _MSC_VER
#pragma warning( pop )
#endif

/* Hier definieren wir unsere 3 Properties */
enum edge_interference_property_t { edge_interference_property };
enum vertex_geometry_property_t { vertex_geometry_property };
enum vertex_id_property_t { vertex_id_property };
enum vertex_component_id_property_t { vertex_component_id_property };

/* Hier installieren wir unsere 3 Properties */
namespace boost {
  BOOST_INSTALL_PROPERTY(edge, interference_property);
  BOOST_INSTALL_PROPERTY(vertex, geometry_property);
  BOOST_INSTALL_PROPERTY(vertex, id_property);  
  BOOST_INSTALL_PROPERTY(vertex, component_id_property);    
}

class GeometrySystem{

public:

  	typedef std::vector< GeometryPtr >::iterator vIterator;
  	typedef std::vector< Interference >::iterator vInterferenceIterator;	
	
	/* Hier werden unsere 4 Properties definiert (vertex_index_t ist standard, deshalb oben nicht dabei)
	 * Die VertexProperties hängen alle aneinander (letztes argument im template ist Vorgänger-Property.)*/
  	typedef boost::property < edge_interference_property_t, InterferencePtr > InterferenceProperty;
	typedef boost::property < boost::vertex_index_t, std::size_t> IndexProperty;
    typedef boost::property < vertex_id_property_t, Id, IndexProperty > IdProperty;
	typedef boost::property < vertex_component_id_property_t, int, IdProperty> ComponentProperty;	
  	typedef boost::property < vertex_geometry_property_t, Geometry*, ComponentProperty> VertexProperty;

	/* Hier definieren wir unseren Graph-Type :
	 * listS vs. vecS : Die Nachbarn sollen als liste gespeichert werden, nicht als vektor
	 * directedS steht für einen gerichtreten Graphen
	 * Die eben definierten Property-Listen VertexProperty und InterferenceProperty werden übergeben*/
  	typedef boost::adjacency_list < boost::listS, boost::listS, boost::directedS, VertexProperty, InterferenceProperty > ContactGraph;

	/* Dies sind die Descriptoren, die einen Vertex bzw. eine Edge beschreiben*/
  	typedef boost::graph_traits < ContactGraph >::vertex_descriptor VertexDescriptor;
  	typedef boost::graph_traits < ContactGraph >::edge_descriptor EdgeDescriptor;	

	/* Typedefs für die Iterator. Dereferenzieren liefert Vertex/Edge-Descriptor */
	typedef boost::graph_traits<ContactGraph>::vertex_iterator VertexIterator;
	typedef boost::graph_traits<ContactGraph>::edge_iterator EdgeIterator;	
	typedef boost::graph_traits<ContactGraph>::out_edge_iterator OutEdgeIter;
	typedef boost::graph_traits<ContactGraph>::in_edge_iterator InEdgeIter;
	
	/* Typedefs für die Property maps */
	typedef boost::property_map<ContactGraph, edge_interference_property_t>::type InterferencePropertyMapType;
	typedef boost::property_map<ContactGraph, vertex_id_property_t>::type IdPropertyMapType;
	typedef boost::property_map<ContactGraph, vertex_component_id_property_t>::type ComponentIdPropertyMapType;
	typedef boost::property_map<ContactGraph, vertex_geometry_property_t>::type GeometryPropertyMapType;	
	
	// TODO umbenennen in IndexPropertyMapType;
    typedef boost::property_map<ContactGraph, boost::vertex_index_t>::type VertexIndexMapType;
	
	
	GeometrySystem();
	virtual ~GeometrySystem();


	//! Macht die KollisionsErkennung mit allen gespeicherten Objekten.
	void resolveCollisions(float interval=0.1);

    static void collisionResponse (GeometryPtr objectA, GeometryPtr objectB,
								   ContactPoint & contactPoint, 
								   bool forCollision, 
								   float collisionRestitution);
	
	void collisionResponse(Interference &interference, 
						   bool forCollision, 
						   float restitution=1.0);

	void printContactGraph();
	void printCollapsedContactGraph();

	//! Hier werden die "Resting Contacts" behandelt.
	void resolveContacts(float interval=0.1); 

	//! hier weden die "Resting Contacts" behandelt. (optimierte version) 
	void resolveContactsFast(float interval=0.1, unsigned int quality = 2); 
	
	//! begin() forward an vCollisionObjects
	vIterator begin() { return vCollisionObjects.begin(); }
	//! end() forward an vCollisionObjects
	vIterator end() { return vCollisionObjects.end(); }
		
	GeometryPtr createSphere(SmartPointer<RigidBody>& rigidBody, float radius);
	GeometryPtr createBox(SmartPointer<RigidBody>& rigidBody, Vector3<float> scale);
	GeometryPtr createPlane(SmartPointer<RigidBody>& rigidBody);
	GeometryPtr createCapsule(SmartPointer<RigidBody>& rigidBody, float radius, float height);

	void airDrag();	
	
	void recheckInterference(InterferencePtr interference);


protected:

	void sendContactInformation(Interference& interference);
	GeometryPtr push_back( GeometryPtr& object);
            
	std::vector< GeometryPtr > vCollisionObjects;
	std::vector< InterferencePtr > mInterferences;

	ContactGraph contactGraph;

	CollisionDetection* mCollisionDetection;
};

#endif // !GEOMETRY_SYSTEM_H
