#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <iostream>
using namespace RoutingKit;
using namespace std;

unsigned userCoordInput(GeoPositionToNode* gp2n)
{

	float outputLongitude, outputLatitude;
	unsigned pointID;
	while(1)
	{
	bool success = false;

	cout << "Enter latitude: ";
	while (!success)
	{
		cin >> outputLatitude;
		if(cin.fail())
		{
			cout << "Wrong format, was expecting float. Try again..." << endl;
		}
		else
		{
			success = true;
			cout << endl;
		}
	}

	success = false;
	cout << "Enter longitude: ";
	while (!success)
	{
		cin >> outputLongitude;
		if(cin.fail())
		{
			cout << "Wrong format, was expecting float. Try again...\n";
		}
		else
		{
			success = true;
			cout << endl;
		}
	}
	pointID = gp2n->find_nearest_neighbor_within_radius(outputLatitude, outputLongitude, 1000).id;
	if(pointID == invalid_id)
		cout << "No node in reasonable distance or outside the map. Try again..." << endl;
	else
		return pointID;
	}

}

int main(){
	// Load a car routing graph from OpenStreetMap-based data
	auto graph = simple_load_osm_car_routing_graph_from_pbf("Turkiye.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

	// Build the shortest path index
	auto ch = ContractionHierarchy::build(
		graph.node_count(), 
		tail, graph.head, 
		graph.travel_time
	);

	// Build the index to quickly map latitudes and longitudes
	GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);

	// Besides the CH itself we need a query object. 
	ContractionHierarchyQuery ch_query(ch);

	// Use the query object to answer queries from stdin to stdout
	unsigned from, to;

	cout << "For the source point:" << endl;
	from = userCoordInput(&map_geo_position);
	cout << "For the destination point:" << endl;
	to = userCoordInput(&map_geo_position);

	long long start_time = get_micro_time();
	ch_query.reset().add_source(from).add_target(to).run();
	auto distance = ch_query.get_distance();
	auto path = ch_query.get_node_path();
	long long end_time = get_micro_time();

	cout << "To get from "<< from << " to "<< to << " one needs " << distance << " milliseconds." << endl;
	cout << "This query was answered in " << (end_time - start_time) << " microseconds." << endl;
	cout << "The path is";
	for(auto x:path)
		cout << " " << x;
	cout << endl;
}