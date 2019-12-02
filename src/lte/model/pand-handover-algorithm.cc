  /* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2013 Budiarto Herman
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
 * Original work authors (from lte-enb-rrc.cc):
 * - Nicola Baldo <nbaldo@cttc.es>
 * - Marco Miozzo <mmiozzo@cttc.es>
 * - Manuel Requena <manuel.requena@cttc.es>
 *
 * Converted to handover algorithm interface by:
 * - Budiarto Herman <budiarto.herman@magister.fi>
 *
 * Modified by: Lucas Pacheco <lucassidpacheco@gmail.com>
 */

#include <math.h>
#include "pand-handover-algorithm.h"
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include "ns3/core-module.h"
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("PandHandoverAlgorithm");

NS_OBJECT_ENSURE_REGISTERED(PandHandoverAlgorithm);

///////////////////////////////////////////
// Handover Management SAP forwarder
///////////////////////////////////////////

PandHandoverAlgorithm::PandHandoverAlgorithm()
    : m_a2MeasId(0)
    , m_a4MeasId(0)
    , m_servingCellThreshold(30)
    , m_neighbourCellOffset(1)
    , m_handoverManagementSapUser(0)
{
    NS_LOG_FUNCTION(this);
    m_handoverManagementSapProvider = new MemberLteHandoverManagementSapProvider<PandHandoverAlgorithm>(this);
}

PandHandoverAlgorithm::~PandHandoverAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
PandHandoverAlgorithm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PandHandoverAlgorithm")
                            .SetParent<LteHandoverAlgorithm>()
                            .SetGroupName("Lte")
                            .AddConstructor<PandHandoverAlgorithm>()
                            .AddAttribute("ServingCellThreshold",
                                "If the RSRQ of the serving cell is worse than this "
                                "threshold, neighbour cells are consider for handover. "
                                "Expressed in quantized range of [0..34] as per Section "
                                "9.1.7 of 3GPP TS 36.133.",
                                UintegerValue(30),
                                MakeUintegerAccessor(&PandHandoverAlgorithm::m_servingCellThreshold),
                                MakeUintegerChecker<uint8_t>(0, 34))
                            .AddAttribute("NumberOfUEs",
                                "Number of UEs in the simulation",
                                UintegerValue(60),
                                MakeUintegerAccessor(&PandHandoverAlgorithm::m_numberOfUEs),
                                MakeUintegerChecker<uint16_t>())
                            .AddAttribute("NeighbourCellOffset",
                                "Minimum offset between the serving and the best neighbour "
                                "cell to trigger the handover. Expressed in quantized "
                                "range of [0..34] as per Section 9.1.7 of 3GPP TS 36.133.",
                                UintegerValue(0),
                                MakeUintegerAccessor(&PandHandoverAlgorithm::m_neighbourCellOffset),
                                MakeUintegerChecker<uint8_t>())
                            .AddAttribute("Threshold",
                                "lorem ipsum",
                                DoubleValue(0.2),
                                MakeDoubleAccessor(&PandHandoverAlgorithm::m_threshold),
                                MakeDoubleChecker<double>())
                            .AddAttribute("TimeToTrigger",
                                "Time during which neighbour cell's RSRP "
                                "must continuously higher than serving cell's RSRP "
                                "in order to trigger a handover",
                                TimeValue(MilliSeconds(256)), // 3GPP time-to-trigger median value as per Section 6.3.5 of 3GPP TS 36.331
                                MakeTimeAccessor(&PandHandoverAlgorithm::m_timeToTrigger),
                                MakeTimeChecker());
    return tid;
}

void PandHandoverAlgorithm::SetLteHandoverManagementSapUser(LteHandoverManagementSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_handoverManagementSapUser = s;
}

LteHandoverManagementSapProvider*
PandHandoverAlgorithm::GetLteHandoverManagementSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_handoverManagementSapProvider;
}

void PandHandoverAlgorithm::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC(this << " requesting Event A4 measurements"
                      << " (threshold=0)");

    LteRrcSap::ReportConfigEutra reportConfig;
    reportConfig.eventId = LteRrcSap::ReportConfigEutra::EVENT_A4;
    reportConfig.threshold1.choice = LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ;
    reportConfig.threshold1.range = 0; // THRESHOLD BAIXO FACILITA DETECÇÃO
    reportConfig.triggerQuantity = LteRrcSap::ReportConfigEutra::RSRQ;
    reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS480;
    m_a4MeasId = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfig);
    LteHandoverAlgorithm::DoInitialize();
}

void PandHandoverAlgorithm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_handoverManagementSapProvider;
}

void PandHandoverAlgorithm::DoReportUeMeas(uint16_t rnti,
    LteRrcSap::MeasResults measResults)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)measResults.measId);

    EvaluateHandover(rnti, measResults.rsrqResult, (uint16_t)measResults.measId, (uint16_t)measResults.servingCellId);

    if (measResults.haveMeasResultNeighCells
        && !measResults.measResultListEutra.empty()) {
        for (std::list<LteRrcSap::MeasResultEutra>::iterator it = measResults.measResultListEutra.begin();
             it != measResults.measResultListEutra.end();
             ++it) {
            NS_ASSERT_MSG(it->haveRsrqResult == true,
                "RSRQ measurement is missing from cellId " << it->physCellId);
            UpdateNeighbourMeasurements(rnti, it->physCellId, it->rsrqResult);
        }
    }
    else {
        NS_LOG_WARN(this << " Event A4 received without measurement results from neighbouring cells");
    }

} // end of DoReportUeMeas

void PandHandoverAlgorithm::EvaluateHandover(uint16_t rnti,
    uint8_t servingCellRsrq, uint16_t measId, uint16_t servingCellId)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)servingCellRsrq);

    // measurements iterator
    // receives rnti and returns the neighbor measurements
    MeasurementTable_t::iterator it1;
    it1 = m_neighbourCellMeasures.find(rnti);

    // quit if end of measurements
    if (it1 == m_neighbourCellMeasures.end()) {
    }
    else {

        // iterate meas for every cell
        MeasurementRow_t::iterator it2;
        // to keep track of cell index in the table
        int cell_iterator = 0;
        int time_converted = Simulator::Now().GetSeconds() * 24 * 3600 /(1000 * 60);

        // find the cell with the largest score (not used here)
        uint16_t bestNeighbourCellId = servingCellId;
        uint16_t bestNeighbourCellRsrq = 0;

        // number of cells
        int n_c = it1->second.size() + 1;
        double cell[it1->second.size() + 1][4];
        double soma[n_c];
        double soma_res = 0;

        //------------ get the node imsi --------------
        int imsi;
        std::stringstream rntiFileName;
        rntiFileName << "./pandora_tmp/" << servingCellId << "/" << rnti;
        std::ifstream rntiFile(rntiFileName.str());
        while (rntiFile >> imsi) {}
        // --------------------------------------------

        // read the transition probability file
        std::stringstream tr_filename;
        tr_filename << "transition_probabilities/" << imsi - 1 << ".txt";
        int a, b;
        double c;
        std::ifstream transitionFile(tr_filename.str().c_str());
        while ( transitionFile >> a >> b >> c){
            if (a == time_converted)
                bestNeighbourCellId = b;
        }
        NS_LOG_INFO("Predicted Cell: " << b);




        // save each cell measurements in a matrix
        // todo remove
        for (it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
            cell[cell_iterator][0] = (uint16_t)it2->second->m_rsrq;
            cell[cell_iterator][3] = it2->first;
            ++cell_iterator;
        }

        // serving cell's meas
        cell[cell_iterator][0] = (uint16_t)servingCellRsrq;
        cell[cell_iterator][3] = servingCellId;

        for (int k = 0; k < n_c; ++k) {
            if (soma[k] > soma_res && cell[k][0] != 0 && cell[k][0] > servingCellRsrq) {
                bestNeighbourCellRsrq = cell[k][0];
                // bestNeighbourCellId = cell[k][3];
                soma_res = soma[k];
            }
        }

        NS_LOG_INFO("\n\n\n------------------------------------------------------------------------");
        NS_LOG_INFO("Measured at: " << time_converted <<
            " Minutes.\nIMSI:" << imsi << "\nRNTI: " << rnti << "\n");
        for (int i = 0; i < n_c; ++i) {
            if (cell[i][3] == servingCellId)
                NS_LOG_INFO("Cell " << cell[i][3] << " -- AHP Score:" << soma[i] << " (serving)");
            else
                NS_LOG_INFO("Cell " << cell[i][3] << " -- AHP Score:" << soma[i]);
            NS_LOG_INFO("         -- RSRQ: " << cell[i][0]);
            // NS_LOG_INFO("         -- MOSp: " << cell[i][1]);
            // NS_LOG_INFO("         -- PDR: " << cell[i][2] << "\n");
        }
        NS_LOG_INFO("\n\nBest Neighbor Cell ID: " << bestNeighbourCellId);


        if (bestNeighbourCellId != servingCellId) {

            std::stringstream outHandFilename;
            outHandFilename << "pandora_tmp/" << "node_" << imsi << "_history";

            std::ifstream inHandFile(outHandFilename.str());

            std::ofstream outHandFile(outHandFilename.str(), std::ofstream::out | std::ofstream::app);
            outHandFile << "\n" << time_converted << " " << bestNeighbourCellId;
            outHandFile.close();

            m_handoverManagementSapUser->TriggerHandover(rnti, bestNeighbourCellId);
            NS_LOG_INFO("Triggering Handover -- RNTI: " << rnti << " -- cellId:" << bestNeighbourCellId << "\n\n\n");
        }
        NS_LOG_INFO("------------------------------------------------------------------------\n\n\n\n");
    }
}

bool PandHandoverAlgorithm::IsValidNeighbour(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);

    return true;
}

void PandHandoverAlgorithm::GetPositions(int imsi, std::string path)
{
    double x, y, z;
    std::string aux1, aux2, aux3, aux4, aux5;
    std::string aux_l1, aux_l2, aux_l3;
    std::ifstream mobilityFile;

    mobilityFile.open(path, std::ios::in);
    if (mobilityFile.is_open()) {
        std::string line;
        while (getline(mobilityFile, line)) {
            std::stringstream at;
            at << "$node_(" << imsi - 1 << ") setdest";
            if (line.find(at.str()) == std::string::npos) {}
            else {
                std::istringstream ss(line);
                ss >> aux1 >> aux2 >> aux3 >> aux4 >> aux5 >> x >> y >> z;
                std::ifstream infile("pandora_tmp/cellsList");
                if (stoi(aux3) >= (int) Simulator::Now().GetSeconds() && stoi(aux3) - Simulator::Now().GetSeconds() < 20){
                    int nc = 0;
                    while(infile >> aux_l1 >> aux_l2 >> aux_l3){
                        m_dist[nc][stoi(aux3)] = sqrt(pow(stod(aux_l2) - x,2) + pow(stod(aux_l3) - y, 2));
                        ++nc;
                    }
                } //
                infile.close();
            }
        }
    }
    mobilityFile.close();

    return;
}

void PandHandoverAlgorithm::UpdateNeighbourMeasurements(uint16_t rnti,
    uint16_t cellId,
    uint8_t rsrq)
{
    NS_LOG_FUNCTION(this << rnti << cellId << (uint16_t)rsrq);
    MeasurementTable_t::iterator it1;
    it1 = m_neighbourCellMeasures.find(rnti);

    if (it1 == m_neighbourCellMeasures.end()) {
        MeasurementRow_t row;
        std::pair<MeasurementTable_t::iterator, bool> ret;
        ret = m_neighbourCellMeasures.insert(std::pair<uint16_t, MeasurementRow_t>(rnti, row));
        NS_ASSERT(ret.second);
        it1 = ret.first;
    }

    NS_ASSERT(it1 != m_neighbourCellMeasures.end());
    Ptr<UeMeasure> neighbourCellMeasures;
    std::map<uint16_t, Ptr<UeMeasure> >::iterator it2;
    it2 = it1->second.find(cellId);

    if (it2 != it1->second.end()) {
        neighbourCellMeasures = it2->second;
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = 0;
        neighbourCellMeasures->m_rsrq = rsrq;
    }
    else {
        neighbourCellMeasures = Create<UeMeasure>();
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = 0;
        neighbourCellMeasures->m_rsrq = rsrq;
        it1->second[cellId] = neighbourCellMeasures;
    }

} // end of UpdateNeighbourMeasurements

} // end of namespace ns3
