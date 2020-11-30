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

#include <list>
#include <unordered_map>
#include <cmath>

using namespace ns3;

unsigned int simTime = 200;
short nTrackers = 2;
short nRelays = 62;
short nVictims = 2;
bool enableNetAnim = false;
list<Ptr<Node>> available_relays;
unordered_map<unsigned int, list<Ptr<Node>>> relay_chains;
float max_tx_radius = 45;
float drone_speed = 10;

void installMobility( NodeContainer firstResponders, NodeContainer drones, NodeContainer victims)
{
	MobilityHelper mobility;
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (firstResponders);

	mobility.SetMobilityModel("ns3::WaypointMobilityModel",
								"InitialPositionIsWaypoint", BooleanValue (true));
	mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
								   "MinX", DoubleValue (-70.0),
								   "MinY", DoubleValue (-70.0),
								   "DeltaX", DoubleValue (20.0),
								   "DeltaY", DoubleValue (20.0),
								   "GridWidth", UintegerValue (8),
								   "LayoutType", StringValue ("RowFirst"));
	mobility.Install (drones);

	mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
								"Mode", StringValue ("Time"),
								"Time", StringValue ("5s"),
								"Speed", StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=8.0]"),
								"Bounds", StringValue ("-1000|1000|-1000|1000"));
	mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
								 "X", StringValue ("ns3::UniformRandomVariable[Min=-1000.0|Max=1000.0]"),
								 "Y", StringValue ("ns3::UniformRandomVariable[Min=-1000.0|Max=1000.0]"),
								 "Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.5]"));
	mobility.Install (victims);
}

ApplicationContainer installApps(Ptr<Node> base, NodeContainer trackers)
{
	ApplicationContainer servers;
	ApplicationContainer clients;
	uint16_t port;
	Ptr<Node> tracker;
	Ipv4Address tracker_address;
	for(uint16_t i = 0; i < nTrackers; ++i)
	{
		port = 8000 + i;
		std::stringstream sdTrace;
		std::stringstream rdTrace;
		sdTrace << "evalvid-logs/sd_a01_" << i;
		rdTrace << "evalvid-logs/rd_a01_" << i;

		tracker = trackers.Get (i);
		EvalvidServerHelper server(port);
		server.SetAttribute ("SenderTraceFilename", StringValue("src/evalvid/st_highway_cif.st"));
		server.SetAttribute ("SenderDumpFilename", StringValue(sdTrace.str()));
		server.SetAttribute ("PacketPayload", UintegerValue(1024));
		servers.Add (server.Install (tracker));

		tracker_address = tracker->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
		EvalvidClientHelper client (tracker_address,port);
		client.SetAttribute ("ReceiverDumpFilename", StringValue(rdTrace.str()));
		clients.Add (client.Install (base));
	}
	servers.Start (Seconds (1));
	servers.Stop (Seconds (simTime-1));
	clients.Stop (Seconds (simTime-1));

	return clients;
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
	NS_LOG_UNCOND("moving drone with nodeId: " << nodeId << " from " << m_position << " to " << destination << " time: " << currentTime);

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

void update_relay_chains(NodeContainer trackers, Ptr<Node> firstResponders){
	double distance;
	double inter_drone_distance;
	double x,y,z;
	Vector fr_position = firstResponders->GetObject<MobilityModel>()->GetPosition();;
	Vector tracker_position;
	Vector relay_position;
	Vector relay_destination;
	Ptr<Node> tracker;
	list<Ptr<Node>>::iterator allocated_relay;
	unsigned int relays_required;
	unsigned int i;

	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		tracker = *iter;
		tracker_position = tracker->GetObject<MobilityModel>()->GetPosition();;
		list<Ptr<Node>>& relay_chain = relay_chains[tracker->GetId()];

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
			move_drone(relay, relay_destination, drone_speed+2);
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
			move_drone(*allocated_relay, relay_destination, drone_speed+2);
			set_wifi_state((*allocated_relay)->GetDevice(0), true);
			++i;
		}
	}
	Simulator::Schedule(Seconds(1), update_relay_chains, trackers, firstResponders);
}

int main  (int argc, char *argv[])
{
	LogComponentEnable ("EvalvidClient", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidServer", LOG_LEVEL_INFO);

	CommandLine cmd;
	cmd.AddValue ("netanim", "Enable generation of NetAnim files", enableNetAnim);
	cmd.Parse (argc, argv);

	NodeContainer firstResponders;
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

	for(auto iter = trackers.Begin(); iter != trackers.End(); ++iter)
	{
		Ptr<Node> tracker = *iter;
		relay_chains[tracker->GetId()] = list<Ptr<Node>>();
	}

	InternetStackHelper internetStack;
	internetStack.Install (nodes);
	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = address.Assign (NetDeviceContainer (meshDevices, relayDevices));

	ApplicationContainer clients;
	clients = installApps(firstResponders.Get(0), trackers);
	clients.Start (Seconds (110));

	move_drone(trackers.Get (0), victims.Get (0)->GetObject<MobilityModel>()->GetPosition(), drone_speed);
	move_drone(trackers.Get (1), victims.Get (1)->GetObject<MobilityModel>()->GetPosition(), drone_speed);
	update_relay_chains(trackers, firstResponders.Get(0));

	AnimationInterface* anim;
	if(enableNetAnim)
	{
		anim = new AnimationInterface("ucam-anim.xml");
		//anim->SetMaxPktsPerTraceFile(2000000); // Set animation interface max packets.
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

	Simulator::Stop (Seconds (simTime));
	Simulator::Run ();
	Simulator::Destroy ();
	return 0;
}
