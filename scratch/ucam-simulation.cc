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

	TrackerInfo()
	{
		streaming_video = false;
	}
};

const float MOBILITY_ENERGY_INTERVAL = 1; //seconds
const float VIDEO_ENERGY_INTERVAL = 2;
const float PREDICTION_ENERGY_INTERVAL = 1;
const float VIDEO_ENERGY_COST = 500; //Joules
const float PREDICTION_ENERGY_COST = 30; //Joules

unsigned int simTime = 200;
short nTrackers = 2;
short nRelays = 62;
short nVictims = 2;
bool enableNetAnim = false;
NodeContainer firstResponders;
list<Ptr<Node>> available_relays;
unordered_map<unsigned int, Ptr<Node>> tracker_by_id;
unordered_map<unsigned int, TrackerInfo> trackers_info;
float max_tx_radius = 50;
float drone_speed = 10;
float drone_height = 30;
std::string ns3_dir;
std::ofstream victims_positions_log;

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
		coordinate = Vector(std::stod(tokens[i+1]), std::stod(tokens[i+2]), drone_height);
		predicted_coords.push_back(coordinate);
	}

	return predicted_coords;
}

void installMobility( NodeContainer firstResponders, NodeContainer drones, NodeContainer victims)
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
								   "MinX", DoubleValue (930.0),
								   "MinY", DoubleValue (930.0),
								   "Z", DoubleValue (drone_height),
								   "DeltaX", DoubleValue (20.0),
								   "DeltaY", DoubleValue (20.0),
								   "GridWidth", UintegerValue (8),
								   "LayoutType", StringValue ("RowFirst"));
	mobility.Install (drones);

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

void UpdateMobilityEnergy(NodeContainer &drones)
{
	bool trigger = false;
	Ptr<EnergySourceContainer> energy_container;

	for (uint16_t i=0 ; i < drones.GetN(); i++)
	{
		Ptr<MobilityModel> drone_position = drones.Get(i)->GetObject<MobilityModel>();

		Vector pos = drone_position->GetPosition ();

		energy_container = drones.Get(i)->GetObject<EnergySourceContainer> ();
		Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(energy_container->Get(0));

		//Ordem de entrada dos parametros: posição X, posição Y, posição Z, tempo de atualização, velocidade
		source->UpdateEnergyMobSource(pos.x,pos.y,pos.z, MOBILITY_ENERGY_INTERVAL, drone_speed);

		float RE = source->GetRemainingEnergy();

		if(RE == 0){
			//trigger = true;
		} 
	}

	if(trigger)
	{
		NodeContainer charg_nodes;

		for (NodeContainer::Iterator j = drones.Begin ();j != drones.End (); ++j)
		{  
			Ptr<Node> object = *j;
			Ptr<MobilityModel> drone_position = object->GetObject<MobilityModel> ();
			Ptr<BasicEnergySource> source = object->GetObject<BasicEnergySource>();

	    	if (source->GetRemainingEnergy() > 0){
				charg_nodes.Add(object);
			}
		}

		drones = charg_nodes;
	}

	//NS_LOG_UNCOND("Numero de nos ativos:");
	//NS_LOG_UNCOND(drones.GetN());
	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &UpdateMobilityEnergy, drones);
}

void update_video_energy(Ptr<Node> tracker){
	Ptr<EnergySourceContainer> energy_container;
	Ptr<BasicEnergySource> source;

	energy_container = tracker->GetObject<EnergySourceContainer>();
	source = DynamicCast<BasicEnergySource>(energy_container->Get(0));
	source->ProcessEnergy(VIDEO_ENERGY_COST);
	trackers_info[tracker->GetId()].video_energy_event = Simulator::Schedule(Seconds(VIDEO_ENERGY_INTERVAL), &update_video_energy, tracker);
}

void update_prediction_energy(Ptr<Node> tracker){
	unsigned int id;
	Ptr<EnergySourceContainer> energy_container;
	Ptr<BasicEnergySource> source;

	id = tracker->GetId();
	energy_container = tracker->GetObject<EnergySourceContainer>();
	source = DynamicCast<BasicEnergySource>(energy_container->Get(0));
	source->ProcessEnergy(PREDICTION_ENERGY_COST);
	trackers_info[id].prediction_energy_event = Simulator::Schedule(Seconds(PREDICTION_ENERGY_INTERVAL), &update_prediction_energy, tracker );
}

void initial_prediction_energy(NodeContainer trackers){
	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		update_prediction_energy(*iter);
	}
}

DeviceEnergyModelContainer installEnergy(NodeContainer drones){
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
	EnergySourceContainer sources = basicSourceHelper.Install (drones);

	// device energy model helper
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// set energy depletion callback
	//WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback callback =
	//MakeCallback (&BasicEnergyDepletionTest::DepletionHandler, this);
	//radioEnergyHelper.SetDepletionCallback (callback);
	Ptr<Node> drone;
	Ptr<MeshPointDevice> droneMeshInterface;
	Ptr<NetDevice> droneInterface;
	NetDeviceContainer devices;
	for(auto iter = drones.Begin(); iter != drones.End(); ++iter)
	{
		drone = *iter;
		droneMeshInterface = DynamicCast<MeshPointDevice> (drone->GetDevice(0));
		droneInterface = droneMeshInterface->GetInterface(1);
		devices.Add(droneInterface);
	}
	
	// install on nodes
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
	return deviceModels;
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

void CourseChange(std::string context, Ptr<const MobilityModel> model)
{
	size_t start = 10;
	size_t end;
	string s_id;
	unsigned int id;
	Ptr<Node> tracker;

	if(model->GetVelocity().GetLength() > 0)
		return;

	end = context.find("/", start);
	s_id = context.substr(start, end - start);
	id = stoul(s_id);
	tracker = tracker_by_id[id];
	TrackerInfo& info = trackers_info[id];

	if(!info.streaming_video)
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
void move_drone(Ptr<Node> drone, Vector destination, double n_vel) {
	// get mobility model for drone
    Ptr<WaypointMobilityModel> mob = drone->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance;

	distance = CalculateDistance(destination, m_position);
	// 1 meter of accuracy is acceptable
	if(distance <= 1)
		return;

	unsigned int nodeId = drone->GetId();
	double currentTime = Simulator::Now().GetSeconds();
	double nWaypointTime;
	NS_LOG_DEBUG("moving drone with nodeId: " << nodeId << " from " << m_position << " to " << destination << " time: " << currentTime);

	mob->EndMobility();
	mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));

	nWaypointTime = distance/n_vel + currentTime;
	mob->AddWaypoint(Waypoint(Seconds(nWaypointTime), destination));
}

list<Ptr<Node>>::iterator closest_drone(Vector coordinate, list<Ptr<Node>>& drones){
	double min;
	double distance;
	Vector drone_position;
	list<Ptr<Node>>::iterator closest;

	auto drone = drones.begin();
	drone_position = (*drone)->GetObject<MobilityModel>()->GetPosition();
	min = CalculateDistance(coordinate, drone_position);
	closest = drone++;
	
	for(; drone != drones.end(); ++drone)
	{
		drone_position = (*drone)->GetObject<MobilityModel>()->GetPosition();
		distance = CalculateDistance(coordinate, drone_position);
		if(distance < min)
		{
			min = distance;
			closest = drone;
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
	x = drone_speed * (destination.x - position.x) / distance;
	y = drone_speed * (destination.y - position.y) / distance;
	z = drone_speed * (destination.z - position.z) / distance;

	future_position.x = position.x + seconds * x;
	future_position.y = position.y + seconds * y;
	future_position.z = position.z + seconds * z;

	return future_position;
}

void update_relay_chains(NodeContainer trackers, Ptr<Node> firstResponders){
	double distance;
	double inter_drone_distance;
	double x,y,z;
	Ptr<WaypointMobilityModel> tracker_mobility;
	Vector fr_position = firstResponders->GetObject<MobilityModel>()->GetPosition();
	Vector tracker_position;
	Vector tracker_destination;
	Vector relay_position;
	Vector relay_destination;
	Ptr<Node> tracker;
	list<Ptr<Node>>::iterator allocated_relay;
	unsigned int relays_required;
	unsigned int i;

	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		tracker = *iter;
		tracker_mobility = tracker->GetObject<WaypointMobilityModel>();
		tracker_position = tracker_mobility->GetPosition();
		tracker_destination = tracker_mobility->GetNextWaypoint().position;
		list<Ptr<Node>>& relay_chain = trackers_info[tracker->GetId()].relay_chain;

		if(CalculateDistance(tracker_destination, tracker_position) > drone_speed)
		{
			NS_LOG_DEBUG("relays may be lagging behind, adjusting relays positions");
			tracker_position = calculate_future_position(tracker_mobility);
		}

		distance = CalculateDistance(fr_position, tracker_position);
		relays_required = (int) ceil(distance / max_tx_radius) - 1;

		//TODO: implement algorithm to free unnecessary relays
		if(relays_required < relay_chain.size())
			relays_required = relay_chain.size();

		inter_drone_distance = distance / (relays_required + 1);
		x = inter_drone_distance * (tracker_position.x - fr_position.x) / distance;
		y = inter_drone_distance * (tracker_position.y - fr_position.y) / distance;
		z = inter_drone_distance * (tracker_position.z - fr_position.z) / distance;
		i = 1;
		for(auto relay : relay_chain)
		{
			relay_destination.x = tracker_position.x - i * x;
			relay_destination.y = tracker_position.y - i * y;
			relay_destination.z = tracker_position.z - i * z;
			move_drone(relay, relay_destination, drone_speed);
			++i;
		}

		while(relay_chain.size() < relays_required && available_relays.size() != 0)
		{
			relay_destination.x = tracker_position.x - i * x;
			relay_destination.y = tracker_position.y - i * y;
			relay_destination.z = tracker_position.z - i * z;
			allocated_relay = closest_drone(relay_destination, available_relays);
			relay_chain.push_back(*allocated_relay);
			available_relays.erase(allocated_relay);
			move_drone(*allocated_relay, relay_destination, drone_speed);
			set_wifi_state((*allocated_relay)->GetDevice(0), true);
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
			victim_position.z = drone_height;
			victims_coords.push_back(victim_position);
		}
	}

	available_trackers.assign (trackers.Begin(), trackers.End());
	for(Vector victim_position : victims_coords)
	{
		nearest_tracker = closest_drone(victim_position, available_trackers);
		move_drone(*nearest_tracker, victim_position, drone_speed);
		available_trackers.erase(nearest_tracker);
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
	NodeContainer drones = NodeContainer(trackers, relays);
	NodeContainer nodes = NodeContainer (firstResponders, drones, victims);

	YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
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
		set_wifi_state (*iter, false);
	}

	installMobility (firstResponders, drones, victims);
	installEnergy (drones);
	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &UpdateMobilityEnergy, drones);
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
	Simulator::Destroy ();
	return 0;
}
