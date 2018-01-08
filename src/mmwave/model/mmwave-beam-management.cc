/*
 * mmwave-beam-management.cc
 *
 *  Created on: 12 oct. 2017
 *      Author: Carlos Herranz
 */


#include "mmwave-beam-management.h"

#include <ns3/log.h>
#include <fstream>
#include <ns3/simulator.h>
#include <ns3/abort.h>
#include <ns3/mmwave-enb-net-device.h>
#include <ns3/mmwave-ue-net-device.h>
#include <ns3/mmwave-ue-phy.h>
#include <ns3/antenna-array-model.h>
#include <ns3/node.h>
#include <algorithm>
#include <ns3/double.h>
#include <ns3/boolean.h>
#include "mmwave-spectrum-value-helper.h"


namespace ns3{

NS_LOG_COMPONENT_DEFINE ("MmWaveBeamManagement");

NS_OBJECT_ENSURE_REGISTERED (MmWaveBeamManagement);




BeamPairInfoStruct::BeamPairInfoStruct()
{
	m_avgSinr = -100;
	m_txBeamId = 65535;
	m_rxBeamId = 65535;
	m_sinrPsd = -1.0;
}


MmWaveBeamManagement::MmWaveBeamManagement()
{
	m_beamSweepParams.m_currentBeamId = 0;
	m_ssBlocksLastBeamSweepUpdate = 0;
}


MmWaveBeamManagement::~MmWaveBeamManagement()
{
	m_enbSinrMap.clear();
}


TypeId
MmWaveBeamManagement::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::MmWaveBeamManagement")
		.SetParent<Object> ()
	;
  	return tid;
}


void
MmWaveBeamManagement::InitializeBeamSweepingTx(Time beamChangeTime)
{
	m_beamSweepParams.m_currentBeamId = 0;
//	std::string txFilePath = "src/mmwave/model/BeamFormingMatrix/TxCodebook.txt";
	std::string txFilePath = "src/mmwave/model/BeamFormingMatrix/KronCodebook16h4v.txt";
	MmWaveBeamforming bfchannel (64,8);
	m_beamSweepParams.m_codebook = LoadCodebookFile(txFilePath);
	this->SetBeamChangeInterval(beamChangeTime);
	m_lastBeamSweepUpdate = Simulator::Now();
	NS_LOG_INFO ("InitializeBeamSweepingTx");
//	DisplayCurrentBeamId();

}

void
MmWaveBeamManagement::InitializeBeamSweepingRx(Time beamChangeTime)
{
	m_beamSweepParams.m_currentBeamId = 0;
//	std::string rxFilePath = "src/mmwave/model/BeamFormingMatrix/RxCodebook.txt";
	std::string rxFilePath = "src/mmwave/model/BeamFormingMatrix/KronCodebook8h2v.txt";
	MmWaveBeamforming bfchannel (64,8);
	m_beamSweepParams.m_codebook = LoadCodebookFile(rxFilePath);
	this->SetBeamChangeInterval(beamChangeTime);
	NS_LOG_INFO ("InitializeBeamSweepingRx");
	m_lastBeamSweepUpdate = Simulator::Now();
//	DisplayCurrentBeamId();

}

void MmWaveBeamManagement::SetBeamChangeInterval (Time period)
{
	m_beamSweepParams.m_steerBeamInterval = period;
}

void MmWaveBeamManagement::SetBeamSweepCodebook (complex2DVector_t codebook)
{
	m_beamSweepParams.m_codebook = codebook;
}

complexVector_t MmWaveBeamManagement::GetBeamSweepVector ()
{
	return m_beamSweepParams.m_codebook.at(m_beamSweepParams.m_currentBeamId);

}

complexVector_t MmWaveBeamManagement::GetBeamSweepVector (uint16_t index)
{
	return m_beamSweepParams.m_codebook.at(index);

}

void MmWaveBeamManagement::BeamSweepStep()
{

	Time currentTime = Simulator::Now();
	uint16_t numCodes = m_beamSweepParams.m_codebook.size();
//	NS_LOG_INFO ("[" << currentTime << "] Beam id " << m_beamSweepParams.m_currentBeamId << " of " << numCodes - 1);
	m_beamSweepParams.m_currentBeamId = (m_beamSweepParams.m_currentBeamId + 1) % numCodes;
	m_lastBeamSweepUpdate = currentTime;
//	NS_LOG_INFO ("[" << currentTime << "] Beam id " << m_beamSweepParams.m_currentBeamId << " of " << numCodes - 1);

}

void MmWaveBeamManagement::BeamSweepStepTx()
{
	Time currentTime = Simulator::Now();
	if (currentTime == 0 || currentTime >= m_lastBeamSweepUpdate + m_beamSweepParams.m_steerBeamInterval)
	{
		BeamSweepStep();
	}
	else
	{
		NS_LOG_INFO ("MmWaveBeamManagement::BeamSweepStepTx() This should not happen");
	}
}

void MmWaveBeamManagement::BeamSweepStepRx()
{
//	Time currentTime = Simulator::Now();
//	if (currentTime >= m_lastBeamSweepUpdate + m_beamSweepParams.m_steerBeamInterval)
	{
		BeamSweepStep();
	}
}


void MmWaveBeamManagement::DisplayCurrentBeamId ()
{
	Time currentTime = Simulator::Now();
	uint16_t numCodes = m_beamSweepParams.m_codebook.size();
	NS_LOG_INFO ("[" << currentTime << "] Beam id " << m_beamSweepParams.m_currentBeamId << " of " << numCodes - 1);
}

uint16_t
MmWaveBeamManagement::GetCurrentBeamId ()
{
	return m_beamSweepParams.m_currentBeamId;
}


uint16_t
MmWaveBeamManagement::GetNumBlocksSinceLastBeamSweepUpdate ()
{
	return m_ssBlocksLastBeamSweepUpdate;
}

uint16_t
MmWaveBeamManagement::IncreaseNumBlocksSinceLastBeamSweepUpdate ()
{
	m_ssBlocksLastBeamSweepUpdate++;
	return m_ssBlocksLastBeamSweepUpdate;

}

void MmWaveBeamManagement::ResetNumBlocksSinceLastBeamSweepUpdate ()
{
	m_ssBlocksLastBeamSweepUpdate = 0;
}

void
MmWaveBeamManagement::AddEnbSinr (Ptr<NetDevice> enbNetDevice, uint16_t enbBeamId, uint16_t ueBeamId, SpectrumValue sinr)
{
	sinrKey key = std::make_pair(enbBeamId,ueBeamId);
	std::map< Ptr<NetDevice>, std::map <sinrKey,SpectrumValue>>::iterator it1 = m_enbSinrMap.find(enbNetDevice);
	if (it1 != m_enbSinrMap.end ())
	{
		std::map <sinrKey,SpectrumValue>::iterator it2 = it1->second.find(key);
		if (it2 != it1->second.end())
		{
			it2->second = sinr;
		}
		else
		{
			it1->second.insert(make_pair(key,sinr));
		}
//		m_enbSinrMap.erase (it1);
	}
	else
	{
		std::map <sinrKey,SpectrumValue> m2;
		m2.insert(make_pair(key,sinr));
		m_enbSinrMap.insert(make_pair(enbNetDevice,m2));
	}

}


BeamPairInfoStruct
MmWaveBeamManagement::GetBestScannedBeamPair ()
{
	BeamPairInfoStruct bestPairInfo;
	double bestAvgSinr = -1.0;

	for (std::map< Ptr<NetDevice>, std::map <sinrKey,SpectrumValue>>::iterator it1 = m_enbSinrMap.begin();
			it1 != m_enbSinrMap.end();
			++it1)
	{
		for (std::map <sinrKey,SpectrumValue>::iterator it2 = it1->second.begin();
				it2 != it1->second.end();
				++it2)
		{
			//FIXME: This comparison does not work. Get the average SINR on the whole bandwidth on each case
			int nbands = it2->second.GetSpectrumModel ()->GetNumBands ();
			double avgSinr = Sum (it2->second)/nbands;
			if (avgSinr > bestAvgSinr)
			{
				bestAvgSinr = avgSinr;
				bestPairInfo.m_avgSinr = avgSinr;
				bestPairInfo.m_sinrPsd = it2->second;
				bestPairInfo.m_targetNetDevice = it1->first;
				bestPairInfo.m_txBeamId = it2->first.first;
				bestPairInfo.m_rxBeamId = it2->first.second;
			}
		}

	}
	return bestPairInfo;
}


void MmWaveBeamManagement::UpdateBestScannedEnb()
{
	BeamPairInfoStruct bestScannedBeamPair = GetBestScannedBeamPair();
//	if (bestScannedBeamPair.m_avgSinr > m_bestScannedEnb.m_avgSinr)
	{
		SetBestScannedEnb(bestScannedBeamPair);
	}
	Time currentTime = Simulator::Now();
	NS_LOG_INFO("[" << currentTime.GetSeconds() <<"]Best beam pair update: tx=" << bestScannedBeamPair.m_txBeamId <<
			" rx=" << bestScannedBeamPair.m_rxBeamId <<
			" avgSinr=" << bestScannedBeamPair.m_avgSinr);
//	return bestScannedBeamPair;
}

std::complex<double>
MmWaveBeamManagement::ParseComplex (std::string strCmplx)
{
    double re = 0.00;
    double im = 0.00;
    size_t findj = 0;
    std::complex<double> out_complex;

    findj = strCmplx.find("i");
    if( findj == std::string::npos )
    {
        im = -1.00;
    }
    else
    {
        strCmplx[findj] = '\0';
    }
    if( ( strCmplx.find("+",1) == std::string::npos && strCmplx.find("-",1) == std::string::npos ) && im != -1 )
    {
        /* No real value */
        re = -1.00;
    }
    std::stringstream stream( strCmplx );
    if( re != -1.00 )
    {
        stream>>re;
    }
    else
    {
        re = 0;
    }
    if( im != -1 )
    {
        stream>>im;
    }
    else
    {
        im = 0.00;
    }
    //  std::cout<<" ---  "<<re<<"  "<<im<<std::endl;
    out_complex = std::complex<double>(re,im);
    return out_complex;
}


complex2DVector_t
MmWaveBeamManagement::LoadCodebookFile (std::string inputFilename)
{
	//std::string filename = "src/mmwave/model/BeamFormingMatrix/TxAntenna.txt";
	complex2DVector_t output;
	NS_LOG_FUNCTION (this << "Loading TxAntenna file " << inputFilename);
	std::ifstream singlefile;
	std::complex<double> complexVar;
	singlefile.open (inputFilename.c_str (), std::ifstream::in);

	NS_LOG_INFO (this << " File: " << inputFilename);
	NS_ASSERT_MSG(singlefile.good (), inputFilename << " file not found");
    std::string line;
    std::string token;
    while( std::getline(singlefile, line) ) //Parse each line of the file
    {
    	complexVector_t txAntenna;
        std::istringstream stream(line);
        while( getline(stream,token,',') ) //Parse each comma separated string in a line
        {
        	complexVar = ParseComplex(token);
		    txAntenna.push_back(complexVar);
		}
        output.push_back(txAntenna);
	}
    return output;
//    NS_LOG_INFO ("TxAntenna[instance:"<<g_enbAntennaInstance.size()<<"][antennaSize:"<<g_enbAntennaInstance[0].size()<<"]");
}

void MmWaveBeamManagement::SetBestScannedEnb(BeamPairInfoStruct bestEnbBeamInfo)
{
	if (bestEnbBeamInfo.m_targetNetDevice)
	{
		m_bestScannedEnb = bestEnbBeamInfo;
	}
}

}

