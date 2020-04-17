/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
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
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LrWpanMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/mobility-module.h>
#include <cmath>


#include <iostream>

using namespace ns3;

uint32_t RECV_STATIC=0;
static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  //NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
  RECV_STATIC++;
}

static void DataConfirm (McpsDataConfirmParams params)
{
 /*
  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
  */
}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
/*
  NS_LOG_UNCOND (context << " state change at " << now.GetSeconds ()              
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState));
*/
}

int main (int argc, char *argv[])
{
  bool verbose 	      = false;
  bool extended 	  = false;
  uint32_t NOD		  = 1000;
  uint32_t Seed	      = 1;

  CommandLine cmd;
  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("extended", "use extended addressing", extended);
  cmd.AddValue ("NOD", "The number of meter devices", NOD);
  cmd.AddValue ("Seed", "Seed number", Seed);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed (Seed);
  LrWpanHelper lrWpanHelper;
  if (verbose)
    {
      lrWpanHelper.EnableLogComponents ();
    }

  // Enable calculation of FCS in the trailers. Only necessary when interacting with real devices or wireshark.
  // GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Create 2 nodes, and a NetDevice for each one
  NodeContainer nodes;
  nodes.Create(NOD);
  //NS_LOG_UNCOND ("end of device configuration1");

  std::vector<Ptr<LrWpanNetDevice>> dev;
  std::vector<Ptr<LrWpanNetDevice>>::iterator it;
  dev.reserve(NOD);
  it=dev.begin();

  for (uint32_t i=0; i<NOD; i++)
  	{
       dev.insert(it, CreateObject<LrWpanNetDevice> ());
    }

  if (!extended)
	 {
	   for (uint32_t i=0; i<NOD; i++)
	   	{
		   dev.at(i)->SetAddress (Mac16Address::Allocate ());
		   std::cout << dev.at(i)->GetMac()->GetShortAddress() << std::endl;
	   	}
	 }
   else
	 {
	   for (uint32_t i=0; i<NOD ;i++)
	 	{
		   dev.at(i)->GetMac()->SetExtendedAddress(Mac64Address::Allocate ());
   		   std::cout << dev.at(i)->GetMac()->GetExtendedAddress() << std::endl;
	 	}
	 }

  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  for (uint32_t i=0; i<NOD; i++)
  	{
		dev.at(i)->SetChannel (channel);
    }

  // To complete configuration, a LrWpanNetDevice must be added to a node
  for (uint32_t i=0; i<NOD; i++)
  	{
  		nodes.Get(i)->AddDevice(dev.at(i));
  	}

  // Trace state changes in the phy
  dev.at(0)->GetPhy ()->TraceConnect ("TrxState", std::string ("phy0"), MakeCallback (&StateChangeNotification));

  //box size per 1 device  
  //double x = 192.5/1000;
  //double y = 180/1000;
  //double z = 102.5/1000;
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
								  "MinX", DoubleValue (100),
								  "MinY", DoubleValue (100),
								  "DeltaX", DoubleValue (192.5/1000),
								  "DeltaY", DoubleValue (180.0/1000),
  								  "Z", DoubleValue (102.5/1000),
								  "GridWidth", UintegerValue (100),
								  "LayoutType", StringValue ("RowFirst"));
   // each object will be attached a static position.
   // i.e., once set by the "position allocator", the
   // position will never change.
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
   // finalize the setup by attaching to each object
   // in the input array a position and initializing
   // this position with the calculated coordinates.
   mobility.Install (nodes);
  
   // iterate our nodes and print their position.
   for (NodeContainer::Iterator j = nodes.Begin ();
		j != nodes.End (); ++j)
	 {
	   Ptr<Node> object = *j;
	   Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
	   NS_ASSERT (position != 0);
	   Vector pos = position->GetPosition ();
	   std::cout << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
	 }

  //Reception packet count of gateway
  McpsDataConfirmCallback cb0;
  cb0 = MakeCallback (&DataConfirm);
  dev[0]->GetMac ()->SetMcpsDataConfirmCallback (cb0);
  McpsDataIndicationCallback cb1;
  cb1 = MakeCallback (&DataIndication);
  dev[0]->GetMac ()->SetMcpsDataIndicationCallback (cb1);

  // Tracing
  lrWpanHelper.EnablePcapAll (std::string ("logdata/lr-wpan-data"), true);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-data.tr");
  lrWpanHelper.EnableAsciiAll (stream);

  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50
  //params.m_txOptions = TX_OPTION_ACK;

  double min = 0.0;
  double max = 3*3600;
  Ptr<UniformRandomVariable> startTimeSeed = CreateObject<UniformRandomVariable> ();
  startTimeSeed->SetAttribute ("Min", DoubleValue (min));
  startTimeSeed->SetAttribute ("Max", DoubleValue (max));

  for (uint32_t i=1; i<NOD; i++)
  	{
	  Ptr<Packet> p0 = Create<Packet> (50);  // 50 bytes of dummy data
	  McpsDataRequestParams params;
	  params.m_dstPanId = 0; 
	  params.m_srcAddrMode = EXT_ADDR;
	  params.m_dstAddrMode = EXT_ADDR;
	  params.m_dstExtAddr = dev.at(0)->GetMac ()->GetExtendedAddress();
	  params.m_msduHandle = 0;
	  //std::cout << uint32_t(startTimeSeed->GetValue()) << std::endl;
	  Simulator::ScheduleWithContext (1, Seconds (uint32_t(startTimeSeed->GetValue())),
	                                  &LrWpanMac::McpsDataRequest,
	                                  dev.at(i)->GetMac (), params, p0);
  	}

  std::cout << NOD << " device and # of device per H,W,L : " << cbrt(NOD) << " : " << ceil(cbrt(NOD)) << std::endl;
  Simulator::Run ();
  std::cout << RECV_STATIC << std::endl;
  Simulator::Destroy ();
  return 0;
}
