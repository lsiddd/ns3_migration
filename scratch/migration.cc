/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Lucas Pacheco <lucas.pacheco@itec.ufpa.br>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/lte-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/csma-module.h"
#include <stdlib.h> /* srand, rand */
// #include "ns3/gtk-config-store.h"
#include <sys/stat.h> // file permissions
// Used for cell allocation
#include <math.h> // sin cos

using namespace ns3;

// function prototipes
void onOffApplication(Ptr<Node>, Ptr<Node>, Ipv4Address, Ipv4Address);
void requestApplication(Ptr<Node>, Ptr<Node>, Ipv4Address, Ipv4Address);
void WriteUntilBufferFull(Ptr<Socket>, uint32_t);
void StartFlow(Ptr<Socket>, Ipv4Address, uint16_t);


float PI = 3.14159265;

// scenario variables
const uint16_t numEnbs = 5;
const uint16_t numNodes = 5;
const uint16_t numEdgeNodes = numEnbs;

// port ranges for each application
static int migrationPort = 1000;
static int pingPort = 2000;
static int applicationPort = 3000;

// use TCP or UDP in the flows
std::string transmissionMode = "UDP";

unsigned int handNumber = 0; // number of handovers executed

// The resources variable tells which server has one or
// more of the recources needed in this simulation
// the resources are:
// base OS (1), base OS + base application(2), or none of these (0)
uint16_t resources[numEdgeNodes];
// units of processing used at the moment
uint16_t serverLoad[numEdgeNodes];

Time managerInterval = MilliSeconds(100);

bool rowTopology = false;
bool randomCellAlloc = true;

// data rate
Time interPacketInterval = MilliSeconds(1);

// control matrix
int cell_ue[numEnbs][numNodes];
// edge servers responsible for each node
int edge_ue[numEdgeNodes][numNodes];

// IP addres of all fog nodes
// the ith server has two addresses, the 7.0.0.X base to communicate with nodes
// and the 22.0.0.X base for migrations
Ipv4Address fogNodesAddresses[numEdgeNodes][2];

// TCP parameters
static uint64_t totalTxBytes[numNodes]; // limit for each node
static uint32_t currentTxBytes = 0;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1000;
uint8_t data[writeSize];

// These are for starting the writing process, and handling the sending
// socket's notification upcalls (events).  These two together more or less
// implement a sending "Application", although not a proper ns3::Application

// subclass.

void NotifyConnectionEstablishedUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": connected to CellId " << cellid << " with RNTI " << rnti << "\n";

    std::stringstream temp_cell_dir;
    std::stringstream ueId;
    temp_cell_dir << "./v2x_temp/" << cellid;
    ueId << temp_cell_dir.str() << "/" << rnti;
    if (mkdir(temp_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(ueId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();

    cell_ue[cellid - 1][imsi - 1] = rnti;
}

void NotifyHandoverStartUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": previously connected to CellId " << cellid << " with RNTI " << rnti << ", doing handover to CellId " << targetCellId << "\n";

    cell_ue[cellid - 1][imsi - 1] = 0;

    std::stringstream ueId;
    ueId << "./v2x_temp/" << cellid << "/" << rnti;
    remove(ueId.str().c_str());

    ++handNumber;
}

void NotifyHandoverEndOkUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": successful handover to CellId " << cellid << " with RNTI " << rnti << "\n";

    std::stringstream target_cell_dir;
    std::stringstream newUeId;
    target_cell_dir << "./v2x_temp/" << cellid;
    newUeId << target_cell_dir.str() << "/" << rnti;
    if (mkdir(target_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(newUeId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();

    cell_ue[cellid - 1][imsi - 1] = rnti;
}

void NotifyConnectionEstablishedEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti << "\n";
}

void NotifyHandoverStartEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId " << targetCellId << "\n";
}

void NotifyHandoverEndOkEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << "\n";
}

void ArrayPositionAllocator(Ptr<ListPositionAllocator> HpnPosition)
{

    std::cout << "allocationg cells positions\n";
    int x, y;
    int distance = 1000;
    std::ofstream outfile("v2x_temp/cellsList", std::ios::out | std::ios::trunc);

    if (randomCellAlloc) {
        std::cout << "random alloc\n";
        for (int i = 0; i < numEnbs; ++i) {
            x = rand() % 100;
            y = rand() % 2000;
            HpnPosition->Add(Vector(x, y, 15));
            outfile << i + 1 << " " << x << " " << y << std::endl;
        }
        outfile.close();
        return;
    }
    else if (rowTopology) {
        std::cout << "row alloc\n";
        int x_start = 700;
        int y_start = 500;
        for (int i = 0; i < numEnbs; ++i)
            HpnPosition->Add(Vector(x_start + distance * i, y_start, 25));
        return;
    }
    else {
        std::cout << "hex alloc\n";
        int x_start = 1000;
        int y_start = 1000;

        HpnPosition->Add(Vector(x_start, y_start, 25));

        for (double i = 0; i < 2 * PI; i += PI / 3) {
            HpnPosition->Add(Vector(x_start + distance * cos(i), y_start + distance * sin(i), 25));
        }

        for (double i = 0; i < 2 * PI; i += PI / 3) {
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 + 10, y_start + distance * sin(i) + rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 + 10, y_start + distance * sin(i) - rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 - 10, y_start + distance * sin(i) + rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 - 10, y_start + distance * sin(i) - rand() % 100 + 10, 10));
        }
    }
}

void manager()
{
    // todo
    std::cout << "manager started at " << Simulator::Now().GetSeconds() << " \n";
    Simulator::Schedule(managerInterval, &manager);
}

void getDelay(Ptr<Node> ueProbeNode, Ptr<Node> edgeProbeNode, Ipv4Address edgeAddress, Ipv4Address ueAddress)
{
    UdpEchoServerHelper echoServer(pingPort);
    std::cout << "pinging on port " << pingPort << "\n";
    pingPort++;

    ApplicationContainer serverApps = echoServer.Install(edgeProbeNode);
    serverApps.Start(Simulator::Now());
    serverApps.Stop(Simulator::Now() + Seconds(1));

    UdpEchoClientHelper echoClient(ueAddress, pingPort);
    echoClient.SetAttribute("MaxPackets", UintegerValue(10));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.2)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(ueProbeNode);
    clientApps.Start(Simulator::Now());
    clientApps.Stop(Simulator::Now() + Seconds(1));
}

void migrate(Ptr<Node> sourceServer,
    Ptr<Node> targetServer,
    Ipv4Address sourceServerAddress,
    Ipv4Address targetServerAddress)
{
    NS_ASSERT_MSG(transmissionMode == "UDP" || transmissionMode == "TCP", "Invalid transmission mode.");
    if (transmissionMode == "UDP") {
        uint16_t port = 4000;
        uint16_t servPort = migrationPort;
        UdpServerHelper server (migrationPort);
        ApplicationContainer apps = server.Install (targetServer);
        apps.Start (Simulator::Now());
        // apps.Stop (Seconds (10.0));

        //
        // Create one UdpClient application to send UDP datagrams from node zero to
        // node one.
        //

        uint32_t MaxPacketSize = 1024;
        Time interPacketInterval = MilliSeconds (1);
        uint32_t maxPacketCount = 320;
        UdpClientHelper client (targetServerAddress, migrationPort);
        client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
        client.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
        apps = client.Install (sourceServer);
        apps.Start (Simulator::Now());
        // apps.Stop (Seconds (10.0));
    }
    else if (transmissionMode == "TCP") {
        uint16_t servPort = migrationPort;
        std::cout << "starting migration on port " << migrationPort << "\n";
        migrationPort++;

        // Create a packet sink to receive these packets on n2...
        PacketSinkHelper sink("ns3::TcpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), servPort));

        ApplicationContainer apps = sink.Install(targetServer);
        apps.Start(Seconds(0.0));

        Ptr<Socket> localSocket = Socket::CreateSocket(sourceServer, TcpSocketFactory::GetTypeId());
        localSocket->Bind();

        // Config::ConnectWithoutContext("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback(&CwndTracer));

        Simulator::ScheduleNow(&StartFlow, localSocket,
            sourceServerAddress, servPort);
    }
}

void stopFlow(int i)
{
    // zero the bytes to be transmitted by the i'th node
    if (totalTxBytes[0] > 0)
        totalTxBytes[0] = 0;
    else
        totalTxBytes[0] = 999999999;
}

void onOffApplication(Ptr<Node> ueNode,
    Ptr<Node> edgeNode,
    Ipv4Address fogNodesAddress,
    Ipv4Address ueIpIface)
{

    uint16_t servPort = applicationPort;
    std::cout << "requesting application on port " << applicationPort << "\n";
    applicationPort++;

    // Create a packet sink to receive these packets on n2...
    PacketSinkHelper sink("ns3::TcpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), servPort));

    ApplicationContainer apps = sink.Install(ueNode);
    apps.Start(Seconds(0.0));

    Ptr<Socket> localSocket = Socket::CreateSocket(edgeNode, TcpSocketFactory::GetTypeId());
    localSocket->Bind();

    // Config::ConnectWithoutContext("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback(&CwndTracer));

    Simulator::ScheduleNow(&StartFlow, localSocket,
        ueIpIface, servPort);
}

//begin implementation of sending "Application"
void StartFlow(Ptr<Socket> localSocket,
    Ipv4Address servAddress,
    uint16_t servPort)
{
    std::cout << "starting flow from " << localSocket;
    localSocket->Connect(InetSocketAddress(servAddress, servPort)); //connect

    // tell the tcp implementation to call WriteUntilBufferFull again
    // if we blocked and new tx buffer space becomes available
    localSocket->SetSendCallback(MakeCallback(&WriteUntilBufferFull));
    WriteUntilBufferFull(localSocket, localSocket->GetTxAvailable());
}

void WriteUntilBufferFull(Ptr<Socket> localSocket, uint32_t txSpace)
{
    // write buffer for the ith node
    // todo: differentiate the flows and return objects
    std::cout << " starting to send.\n";
    while (currentTxBytes < totalTxBytes[0] && localSocket->GetTxAvailable() > 0) {
        uint32_t left = totalTxBytes[0] - currentTxBytes;
        uint32_t dataOffset = currentTxBytes % writeSize;
        uint32_t toWrite = writeSize - dataOffset;
        toWrite = std::min(toWrite, left);
        toWrite = std::min(toWrite, localSocket->GetTxAvailable());
        int amountSent = localSocket->Send(&data[dataOffset], toWrite, 0);
        if (amountSent < 0) {
            // we will be called again when new tx space becomes available.
            return;
        }
        currentTxBytes += amountSent;
    }
    localSocket->Close();
}

int getCellId(int imsi) {
    // start this variable at an arbitrary value
    int cell = 6666;
    for (int i = 0; i < numNodes; ++i) {
            for (int j = 0; j < numEdgeNodes; ++j){
                if (cell_ue[j][i] != 0){
                    std::cout << "user " << i << " connected to cell " << j << "\n";
                    cell = j;
                }
            }
    }
    NS_ASSERT_MSG(cell != 6666, "serving cell not found.");
    // do { (void)sizeof (cell != 6666); } while (false);
    return cell;

}

int main(int argc, char* argv[])
{
    // logs enabled
    // LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
    // LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);
    // LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // random seed
    RngSeedManager::SetSeed(3); // Changes seed from default of 1 to 3

    // initialize the tx buffer.
    for (uint32_t i = 0; i < writeSize; ++i) {
        char m = toascii(97 + i % 26);
        data[i] = m;
    }

    // fill all fog resources with base OS + application files
    std::fill(resources, resources + numEdgeNodes, 2);
    // set the max bytes tranferred by each request
    std::fill(totalTxBytes, totalTxBytes + numEdgeNodes, 9999999990);

    // simulation variables
    Time simTime = Seconds(20);

    // Command line arguments
    CommandLine cmd;

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    cmd.Parse(argc, argv);
    // helpers used
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<EpcHelper> epcHelper;
    epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));

    // ctrl and data channel models
    Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(true));

    // transmission mode (SISO [0], MIMO [1])
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
        UintegerValue(1));
    // path loss model
    lteHelper->SetAttribute("PathlossModel",
        StringValue("ns3::NakagamiPropagationLossModel"));
    // UL and DL frequencies
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetSchedulerType("ns3::PssFfMacScheduler");
    // the maximum number of UE selected by TD scheduler
    lteHelper->SetSchedulerAttribute("nMux", UintegerValue(1));
    // PF scheduler type in PSS
    lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA"));

    // RRC model
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    // Antenna parameters
    lteHelper->SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    lteHelper->SetEnbAntennaModelAttribute("MaxGain", DoubleValue(0.0));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // create internet stack
    InternetStackHelper internet;

    // create users, enbs and fog nodes
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    NodeContainer edgeNodes;
    
    enbNodes.Create(numEnbs);
    ueNodes.Create(numNodes);
    edgeNodes.Create(numEdgeNodes);

    /* edge nodes configuration*/
    internet.Install(edgeNodes);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (int i = 0; i < numEdgeNodes; ++i) {
        // Create the Internet
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
        NetDeviceContainer internetDevices = p2ph.Install(pgw, edgeNodes.Get(i));
        Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
        // interface 0 is localhost, 1 is the p2p device
        fogNodesAddresses[i][0] = internetIpIfaces.GetAddress(1);

        // add network routes to fog nodes
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(edgeNodes.Get(i)->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
        p2ph.EnablePcapAll("lena-simple-epc-backhaul");
    }

    /*-----------------POSIÇÃO DAS TORRES----------------------------------*/
    Ptr<ListPositionAllocator> HpnPosition = CreateObject<ListPositionAllocator>();
    ArrayPositionAllocator(HpnPosition);

    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(edgeNodes);
    remoteHostMobility.Install(pgw);

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(enbNodes);

    // Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/SanFrancisco.tcl");
    Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/carroTrace.tcl");
    MobilityHelper ueMobility;
    MobilityHelper enbMobility;

    ue_mobil.Install(ueNodes.Begin(), ueNodes.End());

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    Ipv4AddressHelper s1uIpv4AddressHelper;
    Ipv4AddressHelper edgeIpv4AddressHelper;

    // Create networks of the S1 interfaces
    s1uIpv4AddressHelper.SetBase("10.0.0.0", "255.255.255.0");
    edgeIpv4AddressHelper.SetBase("22.0.0.0", "255.255.255.0");

    CsmaHelper csma;
    csma.SetChannelAttribute ("DataRate", StringValue ("100Gbps"));
    csma.SetChannelAttribute ("Delay", StringValue ("0ms"));
    NetDeviceContainer d2345 = csma.Install (edgeNodes);
    Ipv4InterfaceContainer edgeIpIfaces = edgeIpv4AddressHelper.Assign(d2345);
    for (int i = 0; i < edgeIpIfaces.GetN(); ++i) {
        fogNodesAddresses[i][1] = edgeIpIfaces.GetAddress(i);
    }
    NS_LOG_UNCOND(edgeIpIfaces.GetAddress(0));

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach one UE per eNodeB
    lteHelper->Attach(ueLteDevs);

    // add x2 interface
    // todo: run handover forecaster
    // with a custom one in this file
    lteHelper->AddX2Interface(enbNodes);

    lteHelper->EnableTraces();
    /*END OF SCENARIO SETUP*/

    // --------------------EVENTS-----------------------------------------

    // //service requests-------------------
    //   for (int i = 0; i < numNodes; ++i)
    //     onOffApplication(ueNodes.Get(i), edgeNodes.Get(i), fogNodesAddresses[i][0], ueIpIface.GetAddress(i));

    //   Simulator::Schedule(Seconds(10), & migrate, edgeNodes.Get(0), edgeNodes.Get(1), fogNodesAddresses[0], fogNodesAddresses[1]);

    //   Simulator::Schedule(Seconds(5), & stopFlow, 0);

    // for (int i = 0; i < numNodes; ++i){
    //     onOffApplication(ueNodes.Get(i), edgeNodes.Get(i), fogNodesAddresses[i], ueIpIface.GetAddress(i));
    //     onOffApplication(edgeNodes.Get(i), ueNodes.Get(i), ueIpIface.GetAddress(i), fogNodesAddresses[i]);
    // }

    // Simulator::Schedule(Seconds(5), &getDelay, ueNodes.Get(0), edgeNodes.Get(0), ueIpIface.GetAddress(0), fogNodesAddresses[0]);
    Simulator::Schedule(Simulator::Now(), &manager);
    Simulator::Schedule(Seconds(5), & migrate, edgeNodes.Get(0), edgeNodes.Get(4), fogNodesAddresses[0][1], fogNodesAddresses[4][1]);

    // netanim setup
    AnimationInterface anim("migration-animation.xml"); // Mandatory

    // callbacks from handover events
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
        MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
        MakeCallback(&NotifyHandoverEndOkUe));


    // intall flow monitor and get stats
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(simTime);
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::cout << "Flow " << i->first << " (" << t.sourceAddress << " ->" << t.destinationAddress << ")\n";
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Lost Packets:   " << i->second.lostPackets << "\n";
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        if (i->second.rxBytes)
            std::cout << "  DelaySum: " << i->second.jitterSum / (i->second.rxPackets + i->second.txPackets) << "\n";
        std::cout << "......................................\n";
    }

    // serialize flow monitor to xml
    flowmon.SerializeToXmlFile("migration_flowmon.xml", true, true);

    Simulator::Destroy();
    return 0;
}