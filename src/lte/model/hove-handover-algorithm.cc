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
#include "hove-handover-algorithm.h"
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include "ns3/core-module.h"
#include <math.h>

namespace ns3 {

using namespace std;

NS_LOG_COMPONENT_DEFINE("HoveHandoverAlgorithm");

NS_OBJECT_ENSURE_REGISTERED(HoveHandoverAlgorithm);

///////////////////////////////////////////
// Handover Management SAP forwarder
///////////////////////////////////////////

HoveHandoverAlgorithm::HoveHandoverAlgorithm(): m_a2MeasId(0), m_a4MeasId(0), m_servingCellThreshold(30), m_neighbourCellOffset(1), m_handoverManagementSapUser(0) {
    NS_LOG_FUNCTION(this);
    m_handoverManagementSapProvider = new MemberLteHandoverManagementSapProvider < HoveHandoverAlgorithm > (this);
}

HoveHandoverAlgorithm::~HoveHandoverAlgorithm() {
    NS_LOG_FUNCTION(this);
}

TypeId
HoveHandoverAlgorithm::GetTypeId() {
    static TypeId tid = TypeId("ns3::HoveHandoverAlgorithm")
        .SetParent < LteHandoverAlgorithm > ()
        .SetGroupName("Lte")
        .AddConstructor < HoveHandoverAlgorithm > ()
        .AddAttribute("mobilityTrace",
            "name of the mobility trace used",
            StringValue("mobil/carroTrace.tcl"),
            MakeStringAccessor( & HoveHandoverAlgorithm::mobilityTrace),
            MakeStringChecker());
    return tid;
}

void HoveHandoverAlgorithm::SetLteHandoverManagementSapUser(LteHandoverManagementSapUser * s) {
    NS_LOG_FUNCTION(this << s);
    m_handoverManagementSapUser = s;
}

LteHandoverManagementSapProvider *
    HoveHandoverAlgorithm::GetLteHandoverManagementSapProvider() {
        NS_LOG_FUNCTION(this);
        return m_handoverManagementSapProvider;
    }

void HoveHandoverAlgorithm::DoInitialize() {
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC(this << " requesting Event A4 measurements" <<
        " (threshold=0)");

    LteRrcSap::ReportConfigEutra reportConfig;
    reportConfig.eventId = LteRrcSap::ReportConfigEutra::EVENT_A4;
    reportConfig.threshold1.choice = LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ;
    reportConfig.threshold1.range = 0; // THRESHOLD BAIXO FACILITA DETECÇÃO
    reportConfig.triggerQuantity = LteRrcSap::ReportConfigEutra::RSRQ;
    reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS480;
    m_a4MeasId = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfig);
    LteHandoverAlgorithm::DoInitialize();
}

void HoveHandoverAlgorithm::DoDispose() {
    NS_LOG_FUNCTION(this);
    delete m_handoverManagementSapProvider;
}

void HoveHandoverAlgorithm::DoReportUeMeas(uint16_t rnti,
    LteRrcSap::MeasResults measResults) {
    NS_LOG_FUNCTION(this << rnti << (uint16_t) measResults.measId);

    EvaluateHandover(rnti, measResults.rsrqResult, (uint16_t) measResults.measId, (uint16_t) measResults.servingCellId);

    if (measResults.haveMeasResultNeighCells &&
        !measResults.measResultListEutra.empty()) {
        for (std::list < LteRrcSap::MeasResultEutra > ::iterator it = measResults.measResultListEutra.begin(); it != measResults.measResultListEutra.end();
            ++it) {
            NS_ASSERT_MSG(it->haveRsrqResult == true,
                "RSRQ measurement is missing from cellId " << it->physCellId);
            UpdateNeighbourMeasurements(rnti, it->physCellId, it->rsrqResult);
        }
    } else {
        NS_LOG_WARN(this << " Event A4 received without measurement results from neighbouring cells");
    }

} // end of DoReportUeMeas

void HoveHandoverAlgorithm::EvaluateHandover(uint16_t rnti,
    uint8_t servingCellRsrq, uint16_t measId, uint16_t servingCellId) {
    NS_LOG_FUNCTION(this << rnti << (uint16_t) servingCellRsrq);

    MeasurementTable_t::iterator it1;
    it1 = m_neighbourCellMeasures.find(rnti);

    if (it1 == m_neighbourCellMeasures.end()) {} else {

        MeasurementRow_t::iterator it2;
        int cell_iterator = 0;

        int imsi;
        std::stringstream rntiFileName;
        rntiFileName << "./v2x_temp/" << servingCellId << "/" << rnti;
        std::ifstream rntiFile(rntiFileName.str());

        uint16_t bestNeighbourCellId = servingCellId;
        uint16_t bestNeighbourCellRsrq = 0;

        int n_c = it1->second.size() + 1;
        double cell[it1->second.size() + 1][4];

        while (rntiFile >> imsi) {}

        for (it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
            cell[cell_iterator][0] = (uint16_t) it2->second->m_rsrq;

            cell[cell_iterator][3] = it2->first;
            ++cell_iterator;
        }

        cell[cell_iterator][0] = (uint16_t) servingCellRsrq;
        cell[cell_iterator][3] = servingCellId;

        std::vector < int > distances = GetPositions(imsi, mobilityTrace, "present", servingCellId);

        // todo: debug why this vector is empty
        // std::vector < int > distances_future = GetPositions(imsi, mobilityTrace, "future", servingCellId);

        // tmp variable to store the lowest distance in the iteration

        double distance_tmp;

        // id of the closest cell
        // int closestCellId;
        for (int i = 0; i < n_c; ++i) {
            if (distances[cell[i][3] - 1] < distance_tmp)
                bestNeighbourCellId = cell[i][3];
        }

        NS_LOG_INFO("\n\nBest Neighbor Cell ID: " << bestNeighbourCellId);

        if (bestNeighbourCellId != servingCellId && bestNeighbourCellRsrq > servingCellRsrq) {

            std::stringstream outHandFilename;
            outHandFilename << "v2x_temp/" << "node_" << imsi << "_history";

            std::ifstream inHandFile(outHandFilename.str());

            NS_LOG_DEBUG("File: " << outHandFilename.str() << " status: " << inHandFile.is_open());

            float a = 0, b = 0, a_old = 0, b_old = 0;
            while (inHandFile >> a >> b) {
                if (inHandFile.eof()) {
                    break;
                }
                a_old = a;
                b_old = b;
            }

            NS_LOG_DEBUG("a: " << a << " b: " << b);
            NS_LOG_DEBUG("a_old: " << a_old << " b_old: " << b_old);
            NS_LOG_DEBUG("best cell: " << bestNeighbourCellId);

            NS_LOG_DEBUG("is_ping_pong: " << Simulator::Now().GetSeconds() - a_old);
            if (bestNeighbourCellId == b_old && Simulator::Now().GetSeconds() - a_old < 4)
                return;

            std::ofstream outHandFile(outHandFilename.str(), std::ofstream::out | std::ofstream::app);
            outHandFile << "\n" << Simulator::Now().GetSeconds() << " " << bestNeighbourCellId;
            outHandFile.close();

            m_handoverManagementSapUser->TriggerHandover(rnti, bestNeighbourCellId);
            NS_LOG_INFO("Triggering Handover -- RNTI: " << rnti << " -- cellId:" << bestNeighbourCellId << "\n\n\n");
        }
        NS_LOG_INFO("------------------------------------------------------------------------\n\n\n\n");
    }
}

bool HoveHandoverAlgorithm::IsValidNeighbour(uint16_t cellId) {
    NS_LOG_FUNCTION(this << cellId);

    return true;
}

std::vector < int > HoveHandoverAlgorithm::GetPositions(int imsi, std::string path, std::string time, int servingCellId) {
    // path = the mobility file being used
    // imsi = the user to be predicted = nodeid - 1

    // coordinate variables
    double node_x, node_y, node_z, node_position_time, cell_y_coord, cell_x_coord;
    int seconds_to_consider = 5;
    std::vector < int > distances;
    std::vector < int > distances_future;

    // temp value to store the value of the distance between the node and a cell
    double dist_value_tmp = 0;

    // tmp veriables to read file
    // node_position_time = time of the position
    std::string aux1, aux2, aux4, aux5;
    std::string cell_id;

    // file stream
    std::ifstream mobilityFile;
    std::ifstream cellFile("v2x_temp/cellsList");

    mobilityFile.open(path, std::ios:: in );
    if (mobilityFile.is_open()) { // if file is open
        std::string line;
        while (getline(mobilityFile, line)) { // get all lines
            std::stringstream at;

            // match all lines that correspont to the current imsi
            at << "$node_(" << imsi - 1 << ") setdest";
            if (!( line.find(at.str()) == std::string::npos ) ) {
                // break line into variables (coordinates and tmp)
                std::istringstream ss(line);
                ss >> aux1 >> aux2 >> node_position_time >> aux4 >> aux5 >> node_x >> node_y >> node_z;

                if (time == "present") {
                    // ignore past values and get future ones
                    if ( round (node_position_time) >=  round(Simulator::Now().GetSeconds())) {
                        // ignore values past seconds_to_consider seconds
                        int nc = 0;
                        while (cellFile >> cell_id >> cell_x_coord >> cell_y_coord) {
                            dist_value_tmp = sqrt(pow(cell_x_coord - node_x, 2) + pow(cell_y_coord - node_y, 2));
                            distances.push_back(dist_value_tmp);
                            ++nc;
                        }
                    }
                } // if time present
                else if (time == "future") {
                    // get current values
                    if ( round(node_position_time) >=  round(Simulator::Now().GetSeconds()) &&
                         round(node_position_time) ==  round(Simulator::Now().GetSeconds())) {
                        // ignore values past seconds_to_consider seconds
                        int nc = 0;
                        while (cellFile >> cell_id >> cell_x_coord >> cell_y_coord) {
                            distances_future.push_back(sqrt(pow(cell_x_coord - node_x, 2) + pow(cell_y_coord - node_y, 2)));
                            NS_LOG_INFO("Node " << imsi << " " << distances_future[nc] << " from cell " << nc << ".\n");
                            ++nc;
                        }
                    }
                } // if time future
                else
                    NS_FATAL_ERROR("Invalid argument to GetPositions");
            } // if line matches user
        } // get lines from mobility file
    } // if file is open

    cellFile.close();
    mobilityFile.close();

    if (time == "present")
        return distances;
    else
        return distances_future;
}

void HoveHandoverAlgorithm::UpdateNeighbourMeasurements(uint16_t rnti,
    uint16_t cellId,
    uint8_t rsrq) {
    NS_LOG_FUNCTION(this << rnti << cellId << (uint16_t) rsrq);
    MeasurementTable_t::iterator it1;
    it1 = m_neighbourCellMeasures.find(rnti);

    if (it1 == m_neighbourCellMeasures.end()) {
        MeasurementRow_t row;
        std::pair < MeasurementTable_t::iterator, bool > ret;
        ret = m_neighbourCellMeasures.insert(std::pair < uint16_t, MeasurementRow_t > (rnti, row));
        NS_ASSERT(ret.second);
        it1 = ret.first;
    }

    NS_ASSERT(it1 != m_neighbourCellMeasures.end());
    Ptr < UeMeasure > neighbourCellMeasures;
    std::map < uint16_t, Ptr < UeMeasure > > ::iterator it2;
    it2 = it1->second.find(cellId);

    if (it2 != it1->second.end()) {
        neighbourCellMeasures = it2->second;
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = 0;
        neighbourCellMeasures->m_rsrq = rsrq;
    } else {
        neighbourCellMeasures = Create < UeMeasure > ();
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = 0;
        neighbourCellMeasures->m_rsrq = rsrq;
        it1->second[cellId] = neighbourCellMeasures;
    }

} // end of UpdateNeighbourMeasurements

} // end of namespace ns3
