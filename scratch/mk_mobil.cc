/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* * Copyright (c) 2018
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
 * Author: Lucas Pacheco  <lucassidpacheco@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-address.h"
#include "ns3/mobility-model.h"
// NetAnim & Evalvid
#include "ns3/netanim-module.h"
#include "ns3/evalvid-client-server-helper.h"
// Pacotes LTE
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"

#include "ns3/string.h"
#include "ns3/double.h"
#include <ns3/boolean.h>
#include <ns3/enum.h>
#include "ns3/flow-monitor-helper.h"

#include <iomanip>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <typeinfo>

// Used for cell allocation
#include <math.h>
#define PI 3.14159265

#define SIMULATION_TIME_FORMAT(s) Seconds(s)

using namespace ns3;

double TxRate = 0;
bool useCbr = false;
bool verbose = false;

const int node_ue = 24;

const uint16_t enb_HPN = 10;
const uint16_t low_power = 0;
const uint16_t hot_spot = 0;

const int node_enb = enb_HPN + low_power + hot_spot;

uint16_t n_cbr = useCbr?enb_HPN+low_power:0;

int hpnTxPower = 46;
int lpnTxPower = 23;
int hpTxPower  = 15;

double simTime = 10;

unsigned int handNumber = 0;

bool randomCellAlloc = true;
bool rowTopology = false;

//coeficiente da média exponencial
unsigned int exp_mean_window = 3;
double qosLastValue = 0;
double qoeLastValue = 0;
int evalvidId = 0;

double qoeSum[enb_HPN + low_power + hot_spot];
double qosSum[enb_HPN + low_power + hot_spot];
int qosMetricsIterator[enb_HPN + low_power + hot_spot];
int qoeMetricsIterator[enb_HPN + low_power + hot_spot];

NS_LOG_COMPONENT_DEFINE("v2x_3gpp");

/*------------------------- NOTIFICAÇÕES DE HANDOVER ----------------------*/
void NotifyConnectionEstablishedUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": connected to CellId " << cellid << " with RNTI " << rnti);

    std::stringstream temp_cell_dir;
    std::stringstream ueId;
    temp_cell_dir << "./v2x_temp/" <<  cellid;
    ueId << temp_cell_dir.str() << "/" << rnti;
    if (mkdir(temp_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(ueId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();
}

void NotifyHandoverStartUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": previously connected to CellId " << cellid << " with RNTI "
        << rnti << ", doing handover to CellId " << targetCellId);

    std::stringstream ueId;
    ueId << "./v2x_temp/" <<  cellid << "/" << rnti;
    remove(ueId.str().c_str());

    ++handNumber;
}

void NotifyHandoverEndOkUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": successful handover to CellId " << cellid << " with RNTI "
        << rnti);


    std::stringstream target_cell_dir;
    std::stringstream newUeId;
    target_cell_dir << "./v2x_temp/" <<  cellid;
    newUeId << target_cell_dir.str() << "/" << rnti;
    if (mkdir(target_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(newUeId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();
}

void NotifyConnectionEstablishedEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": successful connection of UE with IMSI " << imsi << " RNTI "
        << rnti);
}

void NotifyHandoverStartEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": start handover of UE with IMSI " << imsi << " RNTI "
        << rnti << " to CellId " << targetCellId);
}

void NotifyHandoverEndOkEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": completed handover of UE with IMSI " << imsi << " RNTI "
        << rnti);
}

void ArrayPositionAllocator(Ptr<ListPositionAllocator> HpnPosition)
{
    std::ifstream pos_file("CellsDataset");
    int cellId;
    double x_coord, y_coord;

    while (pos_file >> cellId >> x_coord >> y_coord)
    {
      NS_LOG_INFO("Adding cell " << cellId <<
        " to coordinates x:" << x_coord << " y:" << y_coord);
      HpnPosition->Add(Vector(x_coord, y_coord, 30));
    }
}



std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

int main(int argc, char* argv[])
{

    // creation of temporary directory for the simulation
    if (mkdir("./pandora_tmp", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }

    // clean tmp directory and add cellsList file
    system("exec rm -rf pandora_tmp/*");
    system("cp cellsList pandora_tmp/");

    // tmp file that exists while this simulation is running
    std::ofstream tmpFileRALLOC("tmpFileRALLOC");
    tmpFileRALLOC << " ";
    tmpFileRALLOC.close();

    int seedValue = 1;
    CommandLine cmm;

    cmm.AddValue("seedValue", "random seed value.", seedValue);
    cmm.AddValue("verbose", "lte stats verbose.", verbose);
    cmm.Parse(argc, argv);
    RngSeedManager::SetSeed(seedValue + 10000); //valor de seed para geração de números aleatórios
    srand(seedValue);


    LogComponentEnable("v2x_3gpp", LOG_LEVEL_DEBUG);
    LogComponentEnable("v2x_3gpp", LOG_LEVEL_INFO);
    LogComponentEnable("EvalvidClient", LOG_LEVEL_INFO);
    if (verbose) {
      LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
      LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
      //LogComponentEnable("LteUePhy", LOG_LEVEL_ALL);
      //LogComponentEnable("LteEnbPhy", LOG_LEVEL_ALL);
      LogComponentEnable("PandHandoverAlgorithm", LOG_LEVEL_INFO);
      LogComponentEnable("PandHandoverAlgorithm", LOG_LEVEL_DEBUG);
    }

    //-------------Parâmetros da simulação
    uint16_t node_remote = 1; // HOST_REMOTO
    // for (double t = transmissionStart; t < simTime; t += 1)
    //     Simulator::Schedule(Seconds(t), &WriteMetrics);
    /*----------------------------------------------------------------------*/

    //*********** CONFIGURAÇÃO LTE ***************//
    // Bandwidth of Dl and Ul in Resource Blocks
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));

    // Modo de transmissão (SISO [0], MIMO [1])
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
        UintegerValue(1));

    /*------------------------- MÓDULOS LTE ----------------------*/
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<PhyStatsCalculator> m_phyStats = CreateObject<PhyStatsCalculator>();

    lteHelper->SetEpcHelper(epcHelper);

    // LTE configuration
    lteHelper->SetSchedulerType("ns3::PssFfMacScheduler");
    lteHelper->SetSchedulerAttribute("nMux", UintegerValue(1)); // the maximum number of UE selected by TD scheduler
    lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA")); // PF scheduler type in PSS
    lteHelper->SetHandoverAlgorithmType("ns3::HoveHandoverAlgorithm");
    lteHelper->EnableTraces();

    // Propagation Parameters
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetAttribute("PathlossModel",
        StringValue("ns3::NakagamiPropagationLossModel"));


    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    // real RRC modeling
    // May cause lost messages
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    //-------------Antenna Parameters
    lteHelper->SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    lteHelper->SetEnbAntennaModelAttribute("MaxGain", DoubleValue(0.0));

    // Remote host creation
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(node_remote);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // Pilha de Internet
    InternetStackHelper internet;
    internet.Install(remoteHost);

    // Cria link Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Address for the network between PGW and Remote Host
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer internetIpIfaces;
    internetIpIfaces = ipv4h.Assign(internetDevices);

    // add route from remote host to UEs
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"), 1);

    // Network nodes creation
    NodeContainer nodes_ue_nc;
    nodes_ue_nc.Create(node_ue);

    NodeContainer cbr_nodes;
    cbr_nodes.Create(n_cbr);

    NodeContainer enbNodes;
    enbNodes.Create(node_enb);

    NodeContainer MECNodes;
    MECNodes.Create(node_enb);

    internet.Install(nodes_ue_nc);
    internet.Install(cbr_nodes);
    internet.Install(MECNodes);

    /*-----------------POSIÇÃO DAS TORRES----------------------------------*/
    Ptr<ListPositionAllocator> HpnPosition = CreateObject<ListPositionAllocator>();
    ArrayPositionAllocator(HpnPosition);

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(enbNodes);

    // remote host mobility (constant)
    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(remoteHost);
    remoteHostMobility.Install(pgw);

    /*-----------------MONILIDADE DAS TORRES --------------*/

    MobilityHelper mobilityCbr;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(cbr_nodes);

    // User Devices mobility
    Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/mobility_25_users.tcl");
    MobilityHelper ueMobility;
    MobilityHelper enbMobility;

    ue_mobil.Install(nodes_ue_nc.Begin(), nodes_ue_nc.End());

    //-------------Instala LTE Devices para cada grupo de nós
    NetDeviceContainer enbLteDevs;
    enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs;
    ueLteDevs = lteHelper->InstallUeDevice(nodes_ue_nc);
    NetDeviceContainer cbrLteDevs;
    cbrLteDevs = lteHelper->InstallUeDevice(cbr_nodes);


    // Assign an ip for user devices
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    Ipv4InterfaceContainer cbrIpFace;
    cbrIpFace = epcHelper->AssignUeIpv4Address(NetDeviceContainer(cbrLteDevs));

    // add routes from user devides to network gateway
    // in lte, the gateway is the pgw node, connected to the remote host
    for (uint32_t u = 0; u < nodes_ue_nc.GetN(); ++u) {
        Ptr<Node> ueNode = nodes_ue_nc.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),
            1);
    }


    // attach mec servers to enodeb's
    PointToPointHelper p2pMEC;
    p2pMEC.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2pMEC.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2pMEC.SetChannelAttribute("Delay", TimeValue(Seconds(0.02)));

    NetDeviceContainer MECDevice;
    Ipv4AddressHelper address;
    address.SetBase ("6.6.6.0", "255.255.255.0");

    for (uint16_t i = 0; i < MECNodes.GetN(); ++i)
    {
        MECDevice = p2pMEC.Install(enbNodes.Get(i), MECNodes.Get(i));
        Ipv4InterfaceContainer interfaces = address.Assign (MECDevice);
    }

    // TODO: wrap this in a method
    uint16_t cbrPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < MECNodes.GetN(); ++u) {

        // get cbr node
        Ptr<Node> ueNode = MECNodes.Get(u);

        // get node address
        Ptr<Ipv4> ipv4 = ueNode->GetObject<Ipv4>();
        Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
        Ipv4Address addri = iaddr.GetLocal();

        // install server on MEC node
        PacketSinkHelper packetSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), cbrPort));
        serverApps.Add(packetSinkHelper.Install(ueNode));
        serverApps.Start(Seconds(1));

        // install client on remote host
        // todo: 
        int load = 1024;
        UdpClientHelper client(addri, cbrPort);
        client.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
        client.SetAttribute("MaxPackets", UintegerValue(66666666));
        client.SetAttribute("PacketSize", UintegerValue(load));
        clientApps.Add(client.Install(nodes_ue_nc.Get(u)));

        clientApps.Start(Seconds(1));
    }


    // attach nodes and add x2 (for signaling) interface to enbs
    for (uint16_t i = 0; i < nodes_ue_nc.GetN(); ++i)
    {
      lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(i));
    }
    lteHelper->AttachToClosestEnb(cbrLteDevs, enbLteDevs);
    lteHelper->AddX2Interface(enbNodes);


    /*----------------NETANIM CONFIGURATION----------------*/
    AnimationInterface anim("pandora_tmp/LTEnormal_v2x.xml");
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(enbNodes.Get(i), "eNb");
        anim.UpdateNodeColor(enbNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < nodes_ue_nc.GetN(); ++i) {
        anim.UpdateNodeDescription(nodes_ue_nc.Get(i), "UE Carro");
        anim.UpdateNodeColor(nodes_ue_nc.Get(i), 255, 0, 0);
    }
    for (uint32_t i = 0; i < cbr_nodes.GetN(); ++i) {
        anim.UpdateNodeDescription(cbr_nodes.Get(i), "CBR");
        anim.UpdateNodeColor(cbr_nodes.Get(i), 0, 255, 0);
    }
        anim.UpdateNodeDescription(remoteHost, "RH");
        anim.UpdateNodeColor(remoteHost, 0, 255, 255);

    Simulator::Stop(SIMULATION_TIME_FORMAT(simTime));

    /*--------------HANDOVER NOTIFICATIONS-------------------------*/
//     Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
//         MakeCallback(&NotifyConnectionEstablishedUe));
//     Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
//         MakeCallback(&NotifyHandoverStartUe));
//     Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
//         MakeCallback(&NotifyHandoverEndOkUe));

    // ----------- FLOW MONITOR --------------
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

    // run the simulation
    Simulator::Run();

    flowMonitor->SerializeToXmlFile("NameOfFile.xml", true, true);

    // delete simulation tmp file
    remove("tmpFileRALLOC");

    Simulator::Destroy();
    return EXIT_SUCCESS;
}
