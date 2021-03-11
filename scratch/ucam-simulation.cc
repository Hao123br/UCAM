/*
 * =====================================================================================
 *
 *       Filename:  ucam-simulation.cc
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  12/10/2020 22:23:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Carlos Rocha, carlosh2340@gmail.com
 *        Company:  Gercom
 *
 * =====================================================================================
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/evalvid-client-server-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/log.h"
#include "ns3/basic-energy-source.h"
#include "ns3/wifi-radio-energy-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/device-energy-model-container.h"
#include "ns3/event-id.h"
#include "ns3/assert.h"

#include <list>
#include <unordered_map>
#include <cmath>
#include <string>
#include <sstream>
#include <memory>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UCAM");

class TrackerInfo {
public:
	bool streaming_video;
	list<Ptr<Node>> relay_chain;
	EventId video_energy_event;
	EventId prediction_energy_event;
	bool is_victim_visible;
	double last_update_time;
	double total_tracking_time;
	Ptr<Node> tracked_victim;

	TrackerInfo()
	{
		streaming_video = false;
		is_victim_visible = false;
		last_update_time = 0;
		total_tracking_time = 0;
	}
};

class UAVEnergyTrace {
	bool is_relay;
public:
	std::ofstream mobility;
	std::ofstream prediction;
	std::ofstream video;
	std::ofstream comms;

	UAVEnergyTrace(bool r)
	:is_relay{r}
	{
		std::string prefix = is_relay ? "relay-" : "tracker-";

		mobility.open(prefix + "mobility-energy.txt", std::ofstream::out | std::ofstream::trunc);
		comms.open(prefix + "comms-energy.txt", std::ofstream::out | std::ofstream::trunc);

		if(!is_relay)
		{
			prediction.open(prefix + "prediction-energy.txt", std::ofstream::out | std::ofstream::trunc);
			video.open(prefix + "video-energy.txt", std::ofstream::out | std::ofstream::trunc);
		}
	}

	~UAVEnergyTrace()
	{
		mobility.close();
		comms.close();

		if(!is_relay)
		{
			prediction.close();
			video.close();
		}
	}
};

enum class EnergyType { mobility, prediction, video, comms};

const float MOBILITY_ENERGY_INTERVAL = 1; //seconds
const float VIDEO_ENERGY_INTERVAL = 2;
const float PREDICTION_ENERGY_INTERVAL = 1;
const float VIDEO_ENERGY_COST = 500; //Joules
const float PREDICTION_ENERGY_COST = 30; //Joules
const bool WIFI_ON = true;
const bool WIFI_OFF = false;
const bool TYPE_RELAY = true;
const bool TYPE_TRACKER = false;

unsigned int simTime = 300;
short nTrackers = 2;
short nRelays = 34;
short nVictims = 2;
bool enableNetAnim = false;
NodeContainer firstResponders;
list<Ptr<Node>> available_relays;
unordered_map<unsigned int, Ptr<Node>> tracker_by_id;
unordered_map<unsigned int, TrackerInfo> trackers_info;
unordered_map<unsigned int, Vector> uav_charge_station;
float max_tx_radius = 90;
float uav_speed = 10;
float uav_height = 30;
std::string ns3_dir;
std::ofstream victims_positions_log;
UAVEnergyTrace tracker_energy_trace(TYPE_TRACKER);
UAVEnergyTrace relay_energy_trace(TYPE_RELAY);

std::string exec(std::string cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

void log_victims_positions(NodeContainer victims, std::ofstream* victims_positions_log){
	double now = Simulator::Now().GetSeconds();
	for (NodeContainer::Iterator i = victims.Begin(); i != victims.End (); ++i){
		Ptr<Node> victim = *i;
		Ptr<MobilityModel> victim_position = victim->GetObject<MobilityModel> ();
		Vector pos = victim_position->GetPosition ();
		*victims_positions_log << now << "," << victim->GetId() << "," << pos.x << "," << pos.y << "\n";
	}
	victims_positions_log->flush();
	Simulator::Schedule(Seconds(1), &log_victims_positions, victims, victims_positions_log);
}

std::vector<Vector> do_predictions(){
	static double last_time;
	static std::vector<Vector> predicted_coords;
	std::istringstream prediction;
	std::string extracted_token;
	std::vector<std::string> tokens;
	Vector coordinate;

	if(last_time == Simulator::Now().GetSeconds())
		return predicted_coords; //return cached prediction

	predicted_coords.clear();
	last_time = Simulator::Now().GetSeconds();
	prediction.str(exec(std::string("python3 ") + ns3_dir + std::string("/prediction.py 2>>prediction_errors.txt")));

	while(getline(prediction, extracted_token, ' '))
	{
		tokens.push_back(extracted_token);
	}

	for(unsigned int i = 0; i < tokens.size()-1; i+=3){
		coordinate = Vector(std::stod(tokens[i+1]), std::stod(tokens[i+2]), uav_height);
		predicted_coords.push_back(coordinate);
	}

	return predicted_coords;
}

void installMobility( NodeContainer firstResponders, NodeContainer uavs, NodeContainer victims)
{
	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	allocator->Add (Vector(1000, 1000, 0));

	MobilityHelper mobility;
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.SetPositionAllocator (allocator);
	mobility.Install (firstResponders);

	mobility.SetMobilityModel("ns3::WaypointMobilityModel",
								"InitialPositionIsWaypoint", BooleanValue (true));
	mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
								   "MinX", DoubleValue (950.0),
								   "MinY", DoubleValue (950.0),
								   "Z", DoubleValue (0),
								   "DeltaX", DoubleValue (20.0),
								   "DeltaY", DoubleValue (20.0),
								   "GridWidth", UintegerValue (6),
								   "LayoutType", StringValue ("RowFirst"));
	mobility.Install (uavs);

	mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
								"Mode", StringValue ("Time"),
								"Time", StringValue ("5s"),
								"Speed", StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=8.0]"),
								"Bounds", StringValue ("0|2000|0|2000"));
	mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
								 "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2000.0]"),
								 "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2000.0]"),
								 "Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.5]"));
	mobility.Install (victims);
}

void write_energy_trace(EnergyType eType, bool is_relay, unsigned int id, double energy_spent, double remaining_energy)
{
	double currentTime = Simulator::Now().GetSeconds();
	UAVEnergyTrace& eTrace = is_relay ? relay_energy_trace : tracker_energy_trace;
	std::ostringstream message;

	message << currentTime
			<< " " << id
			<< " " << energy_spent
			<< " " << remaining_energy
			<< "\n";

	switch(eType)
	{
		case EnergyType::mobility:
			eTrace.mobility << message.str();
			break;

		case EnergyType::prediction:
			eTrace.prediction << message.str();
			break;

		case EnergyType::video:
			eTrace.video << message.str();
			break;

		case EnergyType::comms:
			eTrace.comms << message.str();
			break;
	}
}

Ptr<BasicEnergySource> get_energy_source(Ptr<Node> uav){
	Ptr<EnergySourceContainer> energy_container;
	Ptr<BasicEnergySource> source;

	energy_container = uav->GetObject<EnergySourceContainer> ();
	source = DynamicCast<BasicEnergySource> (energy_container->Get(0));

	return source;
}

void write_comms_energy_trace(NodeContainer uavs, DeviceEnergyModelContainer emodels, bool is_relay)
{
	Ptr<Node> uav;
	unsigned int id;
	Ptr<WifiRadioEnergyModel> wifi_energy_model;
	Ptr<BasicEnergySource> source;
	double energy_spent, remaining_energy;

	NS_ASSERT (uavs.GetN() == emodels.GetN());

	auto uav_iter = uavs.Begin();
	auto emodel_iter = emodels.Begin();
	for (; uav_iter != uavs.End(); uav_iter++, emodel_iter++)
	{
		uav = *uav_iter;
		wifi_energy_model = DynamicCast<WifiRadioEnergyModel> (*emodel_iter);

		id = uav->GetId();

		energy_spent = wifi_energy_model->GetTotalEnergyConsumption ();

		source = get_energy_source(uav);
		remaining_energy = source->GetRemainingEnergy ();

		write_energy_trace(EnergyType::comms, is_relay, id, energy_spent, remaining_energy);
	}
}

void write_track_time_trace(NodeContainer trackers)
{
	std::ofstream track_time_trace;
	Ptr<Node> tracker;
	Ptr<Node> victim;
	unsigned int id;
	TrackerInfo* info;

	track_time_trace.open("track_time_trace.txt", std::ofstream::out | std::ofstream::trunc);

	for (auto iter = trackers.Begin(); iter != trackers.End(); iter++)
	{
		tracker = *iter;
		id = tracker->GetId();
		info = &trackers_info[id];
		victim = info->tracked_victim;
		track_time_trace << id << " " << victim->GetId() << " " << info->total_tracking_time << "\n";
	}

	track_time_trace.close();
}

void UpdateMobilityEnergy(NodeContainer uavs, bool is_relay)
{
	Ptr<Node> uav;
	unsigned int id;
	Vector pos;
	Ptr<BasicEnergySource> source;
	double energy_spent, remaining_energy;

	for (auto iter = uavs.Begin(); iter != uavs.End(); iter++)
	{
		uav = *iter;

		id = uav->GetId();
		pos = uav->GetObject<MobilityModel>()->GetPosition ();

		source = get_energy_source(uav);

		//Ordem de entrada dos parametros: posição X, posição Y, posição Z, tempo de atualização, velocidade
		energy_spent = source->UpdateEnergyMobSource(pos.x,pos.y,pos.z, MOBILITY_ENERGY_INTERVAL, uav_speed);
		remaining_energy = source->GetRemainingEnergy();

		write_energy_trace(EnergyType::mobility, is_relay, id, energy_spent, remaining_energy);
	}

	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &UpdateMobilityEnergy, uavs, is_relay);
}

void update_video_energy(Ptr<Node> tracker){
	unsigned int id;
	Ptr<BasicEnergySource> source;
	double remaining_energy;

	id = tracker->GetId();
	source = get_energy_source(tracker);
	source->ProcessEnergy(VIDEO_ENERGY_COST);

	remaining_energy = source->GetRemainingEnergy();
	write_energy_trace(EnergyType::video, TYPE_TRACKER, id, VIDEO_ENERGY_COST, remaining_energy);

	trackers_info[id].video_energy_event = Simulator::Schedule(Seconds(VIDEO_ENERGY_INTERVAL), &update_video_energy, tracker);
}

void update_prediction_energy(Ptr<Node> tracker){
	unsigned int id;
	Ptr<BasicEnergySource> source;
	double remaining_energy;

	id = tracker->GetId();
	source = get_energy_source(tracker);
	source->ProcessEnergy(PREDICTION_ENERGY_COST);

	remaining_energy = source->GetRemainingEnergy();
	write_energy_trace(EnergyType::prediction, TYPE_TRACKER, id, PREDICTION_ENERGY_COST, remaining_energy);

	trackers_info[id].prediction_energy_event = Simulator::Schedule(Seconds(PREDICTION_ENERGY_INTERVAL), &update_prediction_energy, tracker );
}

void initial_prediction_energy(NodeContainer trackers){
	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		update_prediction_energy(*iter);
	}
}

DeviceEnergyModelContainer installEnergy(NodeContainer uavs){
	/*
	* Create and install energy source and a single basic radio energy model on
	* the node using helpers.
	*/
	// source helper
	BasicEnergySourceHelper basicSourceHelper;
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (350e3));
	// set update interval
	basicSourceHelper.Set ("PeriodicEnergyUpdateInterval",
						 TimeValue (Seconds (1.5)));
	// install source
	EnergySourceContainer sources = basicSourceHelper.Install (uavs);

	// device energy model helper
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// set energy depletion callback
	//WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback callback =
	//MakeCallback (&BasicEnergyDepletionTest::DepletionHandler, this);
	//radioEnergyHelper.SetDepletionCallback (callback);
	Ptr<Node> uav;
	Ptr<MeshPointDevice> uavMeshInterface;
	Ptr<NetDevice> uavInterface;
	NetDeviceContainer devices;
	for(auto iter = uavs.Begin(); iter != uavs.End(); ++iter)
	{
		uav = *iter;
		uavMeshInterface = DynamicCast<MeshPointDevice> (uav->GetDevice(0));
		uavInterface = uavMeshInterface->GetInterface(1);
		devices.Add(uavInterface);
	}
	
	// install on nodes
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
	return deviceModels;
}

Vector2D to_vector2d(Vector v)
{
	Vector2D v_2d;

	v_2d.x = v.x;
	v_2d.y = v.y;

	return v_2d;
}

void update_tracking_time(Ptr<const MobilityModel> model, TrackerInfo& info)
{
	const double tracking_radius = 13; //for a height of 30m
	Vector tracker_position;
	Vector tracked_victim_position;
	double now;
	double distance;
	double interval;

	now = Simulator::Now().GetSeconds();

	tracker_position = model->GetPosition();
	tracked_victim_position = info.tracked_victim->GetObject<MobilityModel> ()->GetPosition ();

	distance = CalculateDistance (to_vector2d (tracker_position), to_vector2d (tracked_victim_position));
	if (info.is_victim_visible)
	{
		interval = now - info.last_update_time;
		info.total_tracking_time += interval;
		if (distance > tracking_radius)
		{
			info.is_victim_visible = false;
		}
	}
	else
	{
		if (distance <= tracking_radius)
		{
			info.is_victim_visible = true;
		}
	}
	info.last_update_time = now;
}

void install_servers(NodeContainer trackers)
{
	ApplicationContainer servers;
	uint16_t id;
	uint16_t port;
	Ptr<Node> tracker;
	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		tracker = *iter;
		id = tracker->GetId();
		port = 8000 + id;
		std::stringstream sdTrace;
		sdTrace << "evalvid-logs/sd_a01_" << id;

		EvalvidServerHelper server(port);
		server.SetAttribute ("SenderTraceFilename", StringValue("src/evalvid/st_highway_cif.st"));
		server.SetAttribute ("SenderDumpFilename", StringValue(sdTrace.str()));
		server.SetAttribute ("PacketPayload", UintegerValue(1024));
		servers.Add (server.Install (tracker));
	}
	servers.Start (Seconds (1));
	servers.Stop (Seconds (simTime-1));
}

void stream_video(Ptr<Node> tracker)
{
	uint16_t id;
	uint16_t port;
	std::stringstream rdTrace;
	Ipv4Address tracker_address;
	ApplicationContainer client;

	id = tracker->GetId();
	port = 8000 + id;
	rdTrace << "evalvid-logs/rd_a01_" << id;

	tracker_address = tracker->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	EvalvidClientHelper client_helper (tracker_address,port);
	client_helper.SetAttribute ("ReceiverDumpFilename", StringValue(rdTrace.str()));
	client = client_helper.Install (firstResponders.Get(0));
	client.Start(Seconds (1)); //delay start to allow mesh protocol to find a route
	client.Stop(Seconds (simTime-1));
}

unsigned int id_from_context(std::string context)
{
	size_t start = 10;
	size_t end;
	string s_id;
	unsigned int id;

	end = context.find("/", start);
	s_id = context.substr(start, end - start);
	id = stoul(s_id);

	return id;
}

void CourseChange(std::string context, Ptr<const MobilityModel> model)
{
	unsigned int id;
	Ptr<Node> tracker;

	id = id_from_context (context);
	TrackerInfo& info = trackers_info[id];

	update_tracking_time(model, info);

	if (model->GetVelocity().GetLength() > 0)
		return;

	tracker = tracker_by_id[id];
	if (!info.streaming_video)
	{
		Simulator::Cancel(info.prediction_energy_event);
		NS_LOG_DEBUG(context << " starting client");
		stream_video(tracker);
		info.video_energy_event = Simulator::Schedule(
											Seconds (VIDEO_ENERGY_INTERVAL),
											&update_video_energy,
											tracker);
		info.streaming_video = true;
	}
}

// move node "smoothly" towards the given position
void move_uav(Ptr<Node> uav, Vector destination, double n_vel) {
	// get mobility model for uav
    Ptr<WaypointMobilityModel> mob = uav->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance;

	distance = CalculateDistance(destination, m_position);
	// 1 meter of accuracy is acceptable
	if(distance <= 1)
		return;

	unsigned int nodeId = uav->GetId();
	double currentTime = Simulator::Now().GetSeconds();
	double nWaypointTime;
	NS_LOG_DEBUG("moving uav with nodeId: " << nodeId << " from " << m_position << " to " << destination << " time: " << currentTime);

	mob->EndMobility();
	mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));

	nWaypointTime = distance/n_vel + currentTime;
	mob->AddWaypoint(Waypoint(Seconds(nWaypointTime), destination));
}

//finds the relay with lowest energy and that is farthest from the first responders
list<Ptr<Node>>::iterator relay_to_free(list<Ptr<Node>>& relay_chain){
	double energy;
	double min_energy;
	list<Ptr<Node>>::iterator selected;
	Ptr<EnergySourceContainer> source_container;
	Ptr<BasicEnergySource> source;

	auto relay = --relay_chain.end();
	source_container = (*relay)->GetObject<EnergySourceContainer>();
	source = DynamicCast<BasicEnergySource>(source_container->Get(0));
	min_energy = source->GetRemainingEnergy();
	selected = relay;

	//we assume that the last relay in the chain is the closest
	//one to the first responders and iterate from the end
	do
	{
		--relay;
		source_container = (*relay)->GetObject<EnergySourceContainer>();
		source = DynamicCast<BasicEnergySource>(source_container->Get(0));
		energy = source->GetRemainingEnergy();
		//if the energy of the current relay is equal to min_energy
		//we select it, because it is farther from the first responders
		//than the previous one
		if(energy <= min_energy)
		{
			min_energy = energy;
			selected = relay;
		}
	} while (relay != relay_chain.begin());

	return selected;
}

list<Ptr<Node>>::iterator closest_uav(Vector coordinate, list<Ptr<Node>>& uavs){
	double min;
	double distance;
	Vector uav_position;
	list<Ptr<Node>>::iterator closest;

	auto uav = uavs.begin();
	uav_position = (*uav)->GetObject<MobilityModel>()->GetPosition();
	min = CalculateDistance(coordinate, uav_position);
	closest = uav++;
	
	for(; uav != uavs.end(); ++uav)
	{
		uav_position = (*uav)->GetObject<MobilityModel>()->GetPosition();
		distance = CalculateDistance(coordinate, uav_position);
		if(distance < min)
		{
			min = distance;
			closest = uav;
		}
	}
	return closest;
}

void set_wifi_state(Ptr<NetDevice> relay, bool state) {
	Ptr<NetDevice> relayInterface = DynamicCast<MeshPointDevice> (relay)->GetInterface(1);
	Ptr<WifiNetDevice> relayDevice = DynamicCast<WifiNetDevice> (relayInterface);
	Ptr<WifiPhy> relayPhy = relayDevice->GetPhy ();
	if (state)
		relayPhy->ResumeFromOff();
	else
		relayPhy->SetOffMode();
}

Vector calculate_future_position(Ptr<WaypointMobilityModel> mobility)
{
	const float seconds = 1;
	double distance;
	double x,y,z;
	Vector destination = mobility->GetNextWaypoint().position;
	Vector position = mobility->GetPosition();
	Vector future_position;

	NS_LOG_INFO("tracker next waypoint: " << destination);

	distance = CalculateDistance(destination, position);
	x = uav_speed * (destination.x - position.x) / distance;
	y = uav_speed * (destination.y - position.y) / distance;
	z = uav_speed * (destination.z - position.z) / distance;

	future_position.x = position.x + seconds * x;
	future_position.y = position.y + seconds * y;
	future_position.z = position.z + seconds * z;

	return future_position;
}

unsigned int calculate_relays_required(Vector fr_position, Vector tracker_position){
	double tdistance, theight, root, relays;

	tdistance = CalculateDistance(to_vector2d(fr_position), to_vector2d(tracker_position));
	theight = tracker_position.z;

	root = sqrt( pow(max_tx_radius, 2) - pow(theight, 2) );
	relays = (tdistance - root)/max_tx_radius;

	return relays>0 ? (int) ceil(relays) : 0;
}

double calculate_relay_spacing(unsigned int nrelays, Vector fr_position, Vector tracker_position){
	double tdistance, theight, root;

	tdistance = CalculateDistance(to_vector2d(fr_position), to_vector2d(tracker_position));
	theight = tracker_position.z;

	//corner case where the last relay can go past the fr_position
	if(tdistance < nrelays * theight)
		return tdistance / nrelays;

	if(nrelays == 1)
	{
		return (pow(tdistance,2) + pow(theight,2)) / (2*tdistance);
	}
	else
	{
		root = sqrt( pow(tdistance,2) - pow(theight,2) * (pow(nrelays,2) - 1) );
		return (tdistance * nrelays - root) / (pow(nrelays,2) - 1);
	}
}

Vector2D subtract_vectors(Vector2D v1, Vector2D v2){
	Vector2D result;

	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;

	return result;
}

Vector2D unit_vector(Vector2D v){
	double lenght;
	Vector2D unit_vector;

	lenght = v.GetLength();
	unit_vector.x = v.x / lenght;
	unit_vector.y = v.y / lenght;

	return unit_vector;
}

void recharge(Ptr<Node> uav){
	Ptr<EnergySourceContainer> source_container;
	Ptr<BasicEnergySource> source;
	
	source_container = uav->GetObject<EnergySourceContainer>();
	source = DynamicCast<BasicEnergySource>(source_container->Get(0));
	source->CallRecharge();
}

void move_to_recharge_station(Ptr<Node> uav, double uav_speed){
	// get mobility model for uav
    Ptr<WaypointMobilityModel> mob = uav->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
	Vector destination = m_position;
	Waypoint new_waypoint;
    double distance;
	double currentTime = Simulator::Now().GetSeconds();
	double nWaypointTime;
	unsigned int nodeId = uav->GetId();

	NS_LOG_DEBUG("moving uav with nodeId: " << nodeId << " from " << m_position << " to recharge station. time: " << currentTime);

	mob->EndMobility();
	mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));

	//Reduce height first to avoid crashing with other uavs
	destination.z = uav_height / 2;
	distance = CalculateDistance(destination, m_position);
	nWaypointTime = distance/uav_speed;
	new_waypoint.time = Seconds(nWaypointTime + currentTime);
	new_waypoint.position = destination;
	mob->AddWaypoint(new_waypoint);

	//Return to charging station
	m_position = destination;
	destination = uav_charge_station[nodeId];
	distance = CalculateDistance(destination, m_position);
	nWaypointTime += distance/uav_speed;
	new_waypoint.time = Seconds(nWaypointTime + currentTime);
	new_waypoint.position = destination;
	mob->AddWaypoint(new_waypoint);

	//after arriving, shutdown communications and recharge
	Simulator::Schedule(Seconds(nWaypointTime), &set_wifi_state, uav->GetDevice(0), WIFI_OFF);
	Simulator::Schedule(Seconds(nWaypointTime), &recharge, uav);
}

void update_relay_chains(NodeContainer trackers, Ptr<Node> firstResponders){
	double relay_spacing;
	double x,y;
	Ptr<WaypointMobilityModel> tracker_mobility;
	Vector fr_position;
	Vector tracker_position;
	Vector tracker_destination;
	Vector relay_destination;
	Vector2D distance;
	Vector2D direction;
	Ptr<Node> tracker;
	list<Ptr<Node>>::iterator selected_relay;
	unsigned int relays_required;
	unsigned int i;

	fr_position = firstResponders->GetObject<MobilityModel>()->GetPosition();
	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		tracker = *iter;
		tracker_mobility = tracker->GetObject<WaypointMobilityModel>();
		tracker_position = tracker_mobility->GetPosition();
		tracker_destination = tracker_mobility->GetNextWaypoint().position;
		list<Ptr<Node>>& relay_chain = trackers_info[tracker->GetId()].relay_chain;

		if(CalculateDistance(tracker_destination, tracker_position) > uav_speed * 2)
		{
			NS_LOG_DEBUG("relays may be lagging behind, adjusting positions");
			tracker_position = calculate_future_position(tracker_mobility);
		}

		relays_required = calculate_relays_required(fr_position, tracker_position);

		//free unnecessary relays
		while(relays_required < relay_chain.size())
		{
			selected_relay = relay_to_free(relay_chain);
			available_relays.push_back(*selected_relay);
			relay_chain.erase(selected_relay);
			move_to_recharge_station(*selected_relay, uav_speed);
		}

		if(relays_required == 0)
			continue;

		relay_spacing = calculate_relay_spacing (relays_required, fr_position, tracker_position);
		distance = subtract_vectors (to_vector2d(tracker_position), to_vector2d(fr_position));
		direction = unit_vector (distance);

		x = relay_spacing * direction.x;
		y = relay_spacing * direction.y;
		i = 1;

		//move relays to their new positions
		for(auto relay : relay_chain)
		{
			relay_destination.x = tracker_position.x - i * x;
			relay_destination.y = tracker_position.y - i * y;
			relay_destination.z = tracker_position.z;
			move_uav(relay, relay_destination, uav_speed);
			++i;
		}

		//add more relays if necessary
		while(relay_chain.size() < relays_required && available_relays.size() != 0)
		{
			relay_destination.x = tracker_position.x - i * x;
			relay_destination.y = tracker_position.y - i * y;
			relay_destination.z = tracker_position.z;
			selected_relay = closest_uav(relay_destination, available_relays);
			relay_chain.push_back(*selected_relay);
			available_relays.erase(selected_relay);
			move_uav(*selected_relay, relay_destination, uav_speed);
			set_wifi_state((*selected_relay)->GetDevice(0), WIFI_ON);
			++i;
		}
	}
	Simulator::Schedule(Seconds(1), update_relay_chains, trackers, firstResponders);
}

void track_victims(NodeContainer victims, NodeContainer trackers)
{
	double now = Simulator::Now().GetSeconds();
	list<Ptr<Node>> available_trackers;
	list<Ptr<Node>>::iterator nearest_tracker;
	Ptr<Node> victim;
	Vector victim_position;
	std::vector<Vector> victims_coords;

	NS_ASSERT(uav_height <= max_tx_radius);

	if(now >= 10)
	{
		victims_coords = do_predictions();
	}
	else
	{
		for(auto iter = victims.Begin(); iter != victims.End(); ++iter)
		{
			victim = *iter;
			victim_position = victim->GetObject<MobilityModel>()->GetPosition();
			victim_position.z = uav_height;
			victims_coords.push_back(victim_position);
		}
	}

	available_trackers.assign (trackers.Begin(), trackers.End());
	auto iter = victims.Begin();
	for(Vector victim_position : victims_coords)
	{
		nearest_tracker = closest_uav(victim_position, available_trackers);
		move_uav(*nearest_tracker, victim_position, uav_speed);
		available_trackers.erase(nearest_tracker);
		victim = *iter++;
		trackers_info[(*nearest_tracker)->GetId()].tracked_victim = victim;
	}
	Simulator::Schedule(Seconds(2), track_victims, victims, trackers);
}

bool IsTopLevelSourceDir (std::string path)
{
	bool haveVersion = false;
	bool haveLicense = false;

	//
	// If there's a file named VERSION and a file named LICENSE in this
	// directory, we assume it's our top level source directory.
	//

	std::list<std::string> files = SystemPath::ReadFiles (path);
	for (std::list<std::string>::const_iterator i = files.begin (); i != files.end (); ++i)
	{
		if (*i == "VERSION")
		{
			haveVersion = true;
		}
		else if (*i == "LICENSE")
		{
			haveLicense = true;
		}
	}

	return haveVersion && haveLicense;
}

std::string GetTopLevelSourceDir (void)
{
	std::string self = SystemPath::FindSelfDirectory ();
	std::list<std::string> elements = SystemPath::Split (self);
	while (!elements.empty ())
	{
		std::string path = SystemPath::Join (elements.begin (), elements.end ());
		if (IsTopLevelSourceDir (path))
		{
			return path;
		}
		elements.pop_back ();
	}
	NS_FATAL_ERROR ("Could not find source directory from self=" << self);
}

int main  (int argc, char *argv[])
{
	uint32_t seedValue = 1234;
	LogComponentEnable ("UCAM", LOG_LEVEL_DEBUG);
	LogComponentEnable ("EvalvidClient", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidServer", LOG_LEVEL_INFO);
	LogComponentEnable ("BasicEnergySource", LOG_LEVEL_DEBUG);

	CommandLine cmd;
	cmd.AddValue ("netanim", "Enable generation of NetAnim files", enableNetAnim);
	cmd.AddValue("seedValue", "random seed value.", seedValue);
	cmd.Parse (argc, argv);

	ns3::RngSeedManager::SetSeed(seedValue); //valor de seed para geração de números aleatórios
	ns3_dir = GetTopLevelSourceDir();
	victims_positions_log.open("victims_positions_log.txt", std::ofstream::out | std::ofstream::trunc);
	victims_positions_log << nVictims << std::endl;

	firstResponders.Create(1);
	NodeContainer trackers;
	trackers.Create(nTrackers);
	NodeContainer relays;
	relays.Create(nRelays);
	NodeContainer victims;
	victims.Create(nVictims);
	NodeContainer uavs = NodeContainer (trackers, relays);
	NodeContainer nodes = NodeContainer (firstResponders, uavs, victims);

	//YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
	YansWifiChannelHelper channel;
	channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	channel.AddPropagationLoss ("ns3::RangePropagationLossModel","MaxRange",DoubleValue(175));
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
	phy.SetChannel (channel.Create ());

	MeshHelper mesh;
	mesh = MeshHelper::Default ();
	mesh.SetStackInstaller ("ns3::Dot11sStack", 
							"Root", Mac48AddressValue (Mac48Address ("00:00:00:00:00:01")));

	//Window when beacon generating starts (uniform random) in seconds
	mesh.SetMacType ("RandomStart", TimeValue (Seconds (0.1)));
	//Set number of interfaces per mesh point - default is single-interface mesh point
	mesh.SetNumberOfInterfaces (1);
	mesh.SetStandard(WIFI_STANDARD_80211ac);
	// Install protocols and return container if MeshPointDevices
	NetDeviceContainer meshDevices;
	meshDevices = mesh.Install (phy, NodeContainer (firstResponders, trackers, victims));
	NetDeviceContainer relayDevices;
	relayDevices = mesh.Install (phy, relays);

	available_relays.assign (relays.Begin(), relays.End());

	//start relays in off state
	for(auto iter = relayDevices.Begin(); iter != relayDevices.End(); ++iter)
	{
		set_wifi_state (*iter, WIFI_OFF);
	}

	installMobility (firstResponders, uavs, victims);

	for(auto iter = uavs.Begin(); iter != uavs.End(); ++iter)
	{
		Ptr<Node> uav = *iter;
		Ptr<MobilityModel> mob = uav->GetObject<MobilityModel>();
		uav_charge_station[uav->GetId()] = mob->GetPosition();
	}

	DeviceEnergyModelContainer tracker_emodels;
	DeviceEnergyModelContainer relay_emodels;
	tracker_emodels = installEnergy (trackers);
	relay_emodels = installEnergy (relays);

	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &UpdateMobilityEnergy, trackers, TYPE_TRACKER);
	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &UpdateMobilityEnergy, relays, TYPE_RELAY);
	Simulator::Schedule(Seconds(PREDICTION_ENERGY_INTERVAL), &initial_prediction_energy, trackers);

	InternetStackHelper internetStack;
	internetStack.Install (nodes);
	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = address.Assign (NetDeviceContainer (meshDevices, relayDevices));

	install_servers(trackers);

	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		Ptr<Node> tracker = *iter;
		tracker_by_id[tracker->GetId()] = tracker;
	}

	Simulator::Schedule(Seconds(1), &log_victims_positions, victims, &victims_positions_log);
	track_victims(victims, trackers);
	update_relay_chains(trackers, firstResponders.Get(0));

	AnimationInterface* anim;
	if(enableNetAnim)
	{
		anim = new AnimationInterface("ucam-anim.xml");
		anim->SetMaxPktsPerTraceFile(2000000); // Set animation interface max packets.
		for (auto iter = firstResponders.Begin(); iter != firstResponders.End(); ++iter)
		{
			Ptr<Node> fr = *iter;
			anim->UpdateNodeDescription (fr, "First Responder");
			anim->UpdateNodeColor (fr, 128, 128, 128);
			anim->UpdateNodeSize (fr->GetId(),5,5);
		}
		for (auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
		{
			Ptr<Node> tr = *iter;
			anim->UpdateNodeDescription (tr, "Tracker");
			anim->UpdateNodeColor (tr, 0, 255, 0);
			anim->UpdateNodeSize (tr->GetId(),5,5);
		}
		for (auto iter = relays.Begin(); iter != relays.End(); ++iter)
		{
			Ptr<Node> rl = *iter;
			anim->UpdateNodeDescription (rl, "Relay");
			anim->UpdateNodeColor (rl, 0, 0, 255);
			anim->UpdateNodeSize (rl->GetId(),5,5);
		}
		for (auto iter = victims.Begin(); iter != victims.End(); ++iter)
		{
			Ptr<Node> vic = *iter;
			anim->UpdateNodeDescription (vic, "Victim");
			anim->UpdateNodeColor (vic, 255, 0, 0);
			anim->UpdateNodeSize (vic->GetId(),5,5);
		}
	}

	unsigned int id;
	std::ostringstream oss;
	for (uint32_t u = 0; u < trackers.GetN(); ++u) {
		id = trackers.Get(u)->GetId();
		oss << "/NodeList/" << id << "/$ns3::MobilityModel/CourseChange";
		Config::Connect(oss.str(), MakeCallback(&CourseChange));
		oss.str("");
	}

	Simulator::Stop (Seconds (simTime));
	Simulator::Run ();

	write_comms_energy_trace (trackers, tracker_emodels, TYPE_TRACKER);
	write_comms_energy_trace (relays, relay_emodels, TYPE_RELAY);
	write_track_time_trace(trackers);

	Simulator::Destroy ();
	return 0;
}
