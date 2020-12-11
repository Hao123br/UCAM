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

#include <list>
#include <unordered_map>
#include <cmath>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UCAM");

unsigned int simTime = 200;
short nTrackers = 2;
short nRelays = 62;
short nVictims = 2;
bool enableNetAnim = false;
NodeContainer firstResponders;
list<Ptr<Node>> available_relays;
unordered_map<unsigned int, list<Ptr<Node>>> relay_chains;
unordered_map<unsigned int, Ptr<Node>> tracker_by_id;
unordered_map<unsigned int, bool> streaming_video;
float max_tx_radius = 50;
float drone_speed = 10;

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

	if(model->GetVelocity().GetLength() > 0)
		return;

	end = context.find("/", start);
	s_id = context.substr(start, end - start);
	id = stoul(s_id);

	if(!streaming_video[id])
	{
		NS_LOG_DEBUG(context << " starting client");
		stream_video(tracker_by_id[id]);
		streaming_video[id] = true;
	}
}

// move node "smoothly" towards the given position
void move_drone(Ptr<Node> drone, Vector destination, double n_vel) {
	// get mobility model for drone
    Ptr<WaypointMobilityModel> mob = drone->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance = CalculateDistance(destination, m_position);
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
		list<Ptr<Node>>& relay_chain = relay_chains[tracker->GetId()];

		if(CalculateDistance(tracker_destination, tracker_position) > drone_speed)
		{
			NS_LOG_DEBUG("relays may be lagging behind, adjusting relays positions");
			tracker_position = calculate_future_position(tracker_mobility);
		}

		distance = CalculateDistance(fr_position, tracker_position);
		relays_required = (int) ceil(distance / max_tx_radius) - 1;
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
	list<Ptr<Node>> available_trackers;
	Ptr<Node> victim;
	list<Ptr<Node>>::iterator nearest_tracker;
	Vector victim_position;

	available_trackers.assign (trackers.Begin(), trackers.End());
	for(auto iter = victims.Begin(); iter != victims.End(); ++iter)
	{
		victim = *iter;
		victim_position = victim->GetObject<MobilityModel>()->GetPosition();
		nearest_tracker = closest_drone(victim_position, available_trackers);
		move_drone(*nearest_tracker, victim_position, drone_speed);
		available_trackers.erase(nearest_tracker);
	}
	Simulator::Schedule(Seconds(2), track_victims, victims, trackers);
}

int main  (int argc, char *argv[])
{
	LogComponentEnable ("UCAM", LOG_LEVEL_DEBUG);
	LogComponentEnable ("EvalvidClient", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidServer", LOG_LEVEL_INFO);

	CommandLine cmd;
	cmd.AddValue ("netanim", "Enable generation of NetAnim files", enableNetAnim);
	cmd.Parse (argc, argv);

	firstResponders.Create(1);
	NodeContainer trackers;
	trackers.Create(nTrackers);
	NodeContainer relays;
	relays.Create(nRelays);
	NodeContainer victims;
	victims.Create(nVictims);
	NodeContainer nodes = NodeContainer (firstResponders, relays, trackers, victims);

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

	installMobility (firstResponders, NodeContainer(relays, trackers), victims);

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
		relay_chains[tracker->GetId()] = list<Ptr<Node>>();
		tracker_by_id[tracker->GetId()] = tracker;
	}

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
