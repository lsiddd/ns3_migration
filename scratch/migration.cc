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
 * Authors: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/lte-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
 // #include "ns3/gtk-config-store.h"

using namespace ns3;

// scenario variables
const uint16_t numEnbs = 2;
const uint16_t numNodes = 2;
const uint16_t numEdgeNodes = numEnbs;

// The resources variable tells which server has one or
// more of the recources needed in this simulation
// the resources are:
// base OS (1), base OS + base application(2), or none of these (0)
uint16_t resources[numEdgeNodes];

// disable or enable traffic
bool disableDl = false;
bool disableUl = false;

// data rate
Time interPacketInterval = MilliSeconds(1);

// IP addres of all fog nodes
Ipv4Address fogNodesAddresses[numEdgeNodes];

// TCP parameters
static const uint64_t totalTxBytes = 200000000;
static uint32_t currentTxBytes = 0;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1000;
uint8_t data[writeSize];

// function prototipes
void onOffApplication(Ptr < Node > ueNode, Ptr < Node > edgeNode, 
    Ipv4Address fogNodesAddress, Ipv4Address ueIpIface);
void requestApplication(Ptr < Node > ueNode, Ptr < Node > edgeNode, 
    Ipv4Address fogNodesAddress, Ipv4Address ueIpIface);
void WriteUntilBufferFull(Ptr < Socket > localSocket, uint32_t txSpace);
void StartFlow(Ptr < Socket > localSocket, Ipv4Address servAddress, uint16_t servPort);
static void CwndTracer(uint32_t oldval, uint32_t newval);


// These are for starting the writing process, and handling the sending 
// socket's notification upcalls (events).  These two together more or less
// implement a sending "Application", although not a proper ns3::Application

// subclass.

void manager() {
    // todo
    std::cout << "manager started at " << Simulator::Now().GetSeconds() << " \n";
    Simulator::Schedule(MilliSeconds(100), & manager);
}

void migrate() {

}

void onOffApplication(Ptr < Node > ueNode, Ptr < Node > edgeNode, Ipv4Address fogNodesAddress, Ipv4Address ueIpIface) {
    uint16_t servPort = 50000;

    // Create a packet sink to receive these packets on n2...
    PacketSinkHelper sink("ns3::TcpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), servPort));

    ApplicationContainer apps = sink.Install(ueNode);
    apps.Start(Seconds(0.0));

    Ptr < Socket > localSocket =
        Socket::CreateSocket(edgeNode, TcpSocketFactory::GetTypeId());
    localSocket-> Bind();

    Config::ConnectWithoutContext("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback( & CwndTracer));

    Simulator::ScheduleNow( & StartFlow, localSocket,
        ueIpIface, servPort);
}

// void requestApplication(Ptr < Node > ueNode, Ptr < Node > edgeNode, Ipv4Address fogNodesAddress, Ipv4Address ueIpIface) {
//     // Install and start applications on UEs and remote host
//     uint16_t dlPort = 1100;
//     uint16_t ulPort = 2000;

//     ApplicationContainer clientApps;
//     ApplicationContainer serverApps;
//     if (!disableDl) {
//         PacketSinkHelper dlPacketSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
//         serverApps.Add(dlPacketSinkHelper.Install(ueNode));

//         UdpClientHelper dlClient(ueIpIface, dlPort);
//         dlClient.SetAttribute("Interval", TimeValue(interPacketInterval));
//         dlClient.SetAttribute("MaxPackets", UintegerValue(5000));
//         clientApps.Add(dlClient.Install(edgeNode));
//     }

//     if (!disableUl) {
//         ++ulPort;
//         PacketSinkHelper ulPacketSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), ulPort));
//         serverApps.Add(ulPacketSinkHelper.Install(edgeNode));
//         UdpClientHelper ulClient(fogNodesAddress, ulPort);
//         ulClient.SetAttribute("Interval", TimeValue(interPacketInterval));
//         ulClient.SetAttribute("MaxPackets", UintegerValue(5000));
//         clientApps.Add(ulClient.Install(ueNode));
//     }
// }


//begin implementation of sending "Application"
void StartFlow(Ptr < Socket > localSocket,
    Ipv4Address servAddress,
    uint16_t servPort) {
    localSocket-> Connect(InetSocketAddress(servAddress, servPort)); //connect

    // tell the tcp implementation to call WriteUntilBufferFull again
    // if we blocked and new tx buffer space becomes available
    localSocket-> SetSendCallback(MakeCallback( & WriteUntilBufferFull));
    WriteUntilBufferFull(localSocket, localSocket-> GetTxAvailable());
}

void WriteUntilBufferFull(Ptr < Socket > localSocket, uint32_t txSpace) {
    while (currentTxBytes < totalTxBytes && localSocket-> GetTxAvailable() > 0) {
        uint32_t left = totalTxBytes - currentTxBytes;
        uint32_t dataOffset = currentTxBytes % writeSize;
        uint32_t toWrite = writeSize - dataOffset;
        toWrite = std::min(toWrite, left);
        toWrite = std::min(toWrite, localSocket-> GetTxAvailable());
        int amountSent = localSocket-> Send( & data[dataOffset], toWrite, 0);
        if (amountSent < 0) {
            // we will be called again when new tx space becomes available.
            return;
        }
        currentTxBytes += amountSent;
    }
    localSocket-> Close();
}


static void CwndTracer(uint32_t oldval, uint32_t newval) {
    std::cout << "Moving cwnd from " << oldval << " to " << newval << "\n";
}

int
main(int argc, char * argv[]) {

    // logs enabled
    LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
    LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);

    // initialize the tx buffer.
    for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }

    // fill all fog resources with base OS + application files
    std::fill(resources, resources + 100, 2);

    // simulation variables
    Time simTime = Seconds(5);
    double distance = 60.0;

    // Command line arguments
    CommandLine cmd;

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    cmd.Parse(argc, argv);

    // helpers used
    Ptr < LteHelper > lteHelper = CreateObject < LteHelper > ();
    Ptr < EpcHelper > epcHelper;
    epcHelper = CreateObject < NoBackhaulEpcHelper > ();
    lteHelper-> SetEpcHelper(epcHelper);

    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));

    // ctrl and data channel models
    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (true));

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
    lteHelper->SetSchedulerAttribute( "nMux", UintegerValue(1));
    // PF scheduler type in PSS
    lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA"));

    // RRC model
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    // Antenna parameters
    lteHelper->SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    lteHelper->SetEnbAntennaModelAttribute("MaxGain", DoubleValue(0.0));

    Ptr < Node > pgw = epcHelper-> GetPgwNode();

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
        fogNodesAddresses[i] = internetIpIfaces.GetAddress(1);

        // add network routes to fog nodes
        // todo: add routes from nodes to fog nodes
        Ptr < Ipv4StaticRouting > remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(edgeNodes.Get(i)-> GetObject < Ipv4 > ());
        remoteHostStaticRouting-> AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
        p2ph.EnablePcapAll("lena-simple-epc-backhaul");
    }

    // todo: add vehicular mobility trace
    // Install Mobility Model for eNBs and UEs
    Ptr < ListPositionAllocator > positionAlloc = CreateObject < ListPositionAllocator > ();
    for (uint16_t i = 0; i < numNodes; i++) {
        positionAlloc-> Add(Vector(distance * i, 0, 0));
    }
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);
    mobility.Install(ueNodes);

    // SGW node
    Ptr < Node > sgw = epcHelper-> GetSgwNode();

    // Install Mobility Model for SGW
    Ptr < ListPositionAllocator > positionAlloc2 = CreateObject < ListPositionAllocator > ();
    positionAlloc2-> Add(Vector(0.0, 50.0, 0.0));
    MobilityHelper mobility2;
    mobility2.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility2.SetPositionAllocator(positionAlloc2);
    mobility2.Install(sgw);

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper-> InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper-> InstallUeDevice(ueNodes);

    Ipv4AddressHelper s1uIpv4AddressHelper;
    Ipv4AddressHelper edgeIpv4AddressHelper;

    // Create networks of the S1 interfaces
    s1uIpv4AddressHelper.SetBase("10.0.0.0", "255.255.255.252");
    edgeIpv4AddressHelper.SetBase("22.0.0.0", "255.255.255.0");

    for (uint16_t i = 0; i < numEnbs; ++i) {
        Ptr < Node > enb = enbNodes.Get(i);

        // Create a point to point link between the eNB and the SGW with
        // the corresponding new NetDevices on each side
        PointToPointHelper p2ph;
        DataRate s1uLinkDataRate = DataRate("10Gb/s");
        uint16_t s1uLinkMtu = 2000;
        Time s1uLinkDelay = Time(0);
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(s1uLinkDataRate));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(s1uLinkMtu));
        p2ph.SetChannelAttribute("Delay", TimeValue(s1uLinkDelay));
        NetDeviceContainer sgwEnbDevices = p2ph.Install(sgw, enb);

        Ipv4InterfaceContainer sgwEnbIpIfaces = s1uIpv4AddressHelper.Assign(sgwEnbDevices);
        s1uIpv4AddressHelper.NewNetwork();

        Ipv4Address sgwS1uAddress = sgwEnbIpIfaces.GetAddress(0);
        Ipv4Address enbS1uAddress = sgwEnbIpIfaces.GetAddress(1);

        // Create S1 interface between the SGW and the eNB
        epcHelper-> AddS1Interface(enb, enbS1uAddress, sgwS1uAddress);
    }

    for (int i = 0; i < numEdgeNodes; ++i) {

        NetDeviceContainer edgeDevices;

        PointToPointHelper p2ph;
        DataRate edgeLinkDataRate = DataRate("10Gb/s");
        uint16_t edgeLinkMtu = 2000;
        Time edgeLinkDelay = Time(0);

        p2ph.SetDeviceAttribute("DataRate", DataRateValue(edgeLinkDataRate));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(edgeLinkMtu));
        p2ph.SetChannelAttribute("Delay", TimeValue(edgeLinkDelay));
        if (i < numEdgeNodes - 1)
            edgeDevices = p2ph.Install(edgeNodes.Get(i), edgeNodes.Get(i+1));

        Ipv4InterfaceContainer edgeIpIfaces = s1uIpv4AddressHelper.Assign(edgeDevices);
        edgeIpv4AddressHelper.NewNetwork();
    }

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper-> AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
        Ptr < Node > ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr < Ipv4StaticRouting > ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode-> GetObject < Ipv4 > ());
        ueStaticRouting-> SetDefaultRoute(epcHelper-> GetUeDefaultGatewayAddress(), 1);
    }

    // Attach one UE per eNodeB
    lteHelper-> Attach(ueLteDevs);

    // add x2 interface
    // todo: replace the handover algorith 
    // with a custom one in this file
    lteHelper-> AddX2Interface(enbNodes);

    lteHelper-> EnableTraces();
    /*END OF SCENARIO SETUP*/

    /*START THE APPLICATIONS AND MANAGER*/
    // requestApplication(ueNodes.Get(0), edgeNodes.Get(0), fogNodesAddresses[0], ueIpIface.GetAddress(0));

    onOffApplication(ueNodes.Get(0), edgeNodes.Get(1), fogNodesAddresses[1], ueIpIface.GetAddress(0));
    // for (int i = 0; i < numNodes; ++i){
    //     onOffApplication(ueNodes.Get(i), edgeNodes.Get(i), fogNodesAddresses[i], ueIpIface.GetAddress(i));
    //     onOffApplication(edgeNodes.Get(i), ueNodes.Get(i), ueIpIface.GetAddress(i), fogNodesAddresses[i]);
    // }


    Simulator::Schedule(Simulator::Now(), & manager);

    // intall flow monitor and get stats
    FlowMonitorHelper flowmon;
    Ptr < FlowMonitor > monitor = flowmon.InstallAll();

    Simulator::Stop(simTime);
    Simulator::Run();

    monitor-> CheckForLostPackets();
    Ptr < Ipv4FlowClassifier > classifier = DynamicCast < Ipv4FlowClassifier > (flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor-> GetFlowStats();
    for (std::map < FlowId, FlowMonitor::FlowStats > ::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier-> FindFlow(i-> first);
        std::cout << "Flow " << i-> first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "  Tx Packets: " << i-> second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i-> second.txBytes << "\n";
        std::cout << "  TxOffered:  " << i-> second.txBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        std::cout << "  Rx Packets: " << i-> second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i-> second.rxBytes << "\n";
        std::cout << "  Lost Packets:   " << i-> second.lostPackets << "\n";
        std::cout << "  Throughput: " << i-> second.rxBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        if (i-> second.rxBytes)
            std::cout << "  DelaySum: " << i-> second.jitterSum / (i-> second.rxPackets + i-> second.txPackets) << "\n";
        std::cout << "......................................\n";
    }

    // serialize flow monitor to xml
    flowmon.SerializeToXmlFile("migration_flowmon.xml", true, true);

    Simulator::Destroy();
    return 0;
}
