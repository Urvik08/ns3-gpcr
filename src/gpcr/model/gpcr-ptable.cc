#include "gpcr-ptable.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("GpcrTable");


namespace ns3 {
namespace gpcr {

/*
  GPCR position table
*/

PositionTable::PositionTable ()
{
  m_txErrorCallback = MakeCallback (&PositionTable::ProcessTxError, this);
  m_entryLifeTime = Seconds (2); //FIXME fazer isto parametrizavel de acordo com tempo de hello

}

uint8_t
PositionTable::AmICoordinator ()
{
  Purge ();
  if (m_table.empty ())
    {
      return 0;
    }//if table is empty (no neighbours)

double covXY;
double covX;
double covY;
double meanX = 0;
double meanY = 0;
int count = 0;
/*according to GPCR authors p_xy = MOD(covXY/(covX*covY)), if p_xy<0.9 node is in junction*/

/*cicle through neighbours to calculate meanX and meanY*/
std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i;
  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
	count++;
	meanX += i->second.first.x;
	meanY += i->second.first.y;
	}
meanX = meanX / count;
meanY = meanY / count;

/*cicle again to calculate covXY, covX and covY*/
  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
	covXY = covXY + ((i->second.first.x - meanX) * (i->second.first.y - meanY));
	covX = covX + ((i->second.first.x - meanX) * (i->second.first.x - meanX));
	covY = covY + ((i->second.first.y - meanY) * (i->second.first.y - meanY));
	}
	covX = sqrt(covX);
	covY = sqrt(covY);
	
	double p_xy = covXY / (covX * covY);
	
	if(p_xy < 0.9 && p_xy > -0.9)
	{
		return 1;
	}
	return 0;

}



Time 
PositionTable::GetEntryUpdateTime (Ipv4Address id)
{
  if (id == Ipv4Address::GetZero ())
    {
      return Time (Seconds (0));
    }
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i = m_table.find (id);
  return i->second.second.first;
}

uint8_t
PositionTable::GetIsCoordinator (Ipv4Address id)
{
  if (id == Ipv4Address::GetZero ())
    {
      return 0;
    }
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i = m_table.find (id);
  return i->second.second.second;
}



/**
 * \brief Adds entry in position table
 */
void 
PositionTable::AddEntry (Ipv4Address id, Vector position, uint8_t isCoordinator)
{
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i = m_table.find (id);
  if (i != m_table.end () || id.IsEqual (i->first))
    {
      m_table.erase (id);
    }
  

  m_table.insert (std::make_pair (id, std::make_pair (position, std::make_pair (Simulator::Now (), isCoordinator))));
}

/**
 * \brief Deletes entry in position table
 */
void PositionTable::DeleteEntry (Ipv4Address id)
{
  m_table.erase (id);
}

/**
 * \brief Gets position from position table
 * \param id Ipv4Address to get position from
 * \return Position of that id or NULL if not known
 */
Vector 
PositionTable::GetPosition (Ipv4Address id) /*FIXME gets from NodeList??? is this correct?*/
{

  NodeList::Iterator listEnd = NodeList::End ();
  for (NodeList::Iterator i = NodeList::Begin (); i != listEnd; i++)
    {
      Ptr<Node> node = *i;
      if (node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () == id)
        {
          return node->GetObject<MobilityModel> ()->GetPosition ();
        }
    }
  return PositionTable::GetInvalidPosition ();

}

/**
 * \brief Checks if a node is a neighbour
 * \param id Ipv4Address of the node to check
 * \return True if the node is neighbour, false otherwise
 */
bool
PositionTable::isNeighbour (Ipv4Address id)
{

 std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i = m_table.find (id);
  if (i != m_table.end () || id.IsEqual (i->first))
    {
      return true;
    }

  return false;
}


/**
 * \brief remove entries with expired lifetime
 */
void 
PositionTable::Purge ()
{

  if(m_table.empty ())
    {
      return;
    }

  std::list<Ipv4Address> toErase;

  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i = m_table.begin ();
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator listEnd = m_table.end ();
  
  for (; !(i == listEnd); i++)
    {

      if (m_entryLifeTime + GetEntryUpdateTime (i->first) <= Simulator::Now ())
        {
          toErase.insert (toErase.begin (), i->first);

        }
    }
  toErase.unique ();

  std::list<Ipv4Address>::iterator end = toErase.end ();

  for (std::list<Ipv4Address>::iterator it = toErase.begin (); it != end; ++it)
    {

      m_table.erase (*it);

    }
}

/**
 * \brief clears all entries
 */
void 
PositionTable::Clear ()
{
  m_table.clear ();
}


/**
 * \brief Gets a coordinator as next hop if thre is one
 * \param position the position of the node that has the packet
 * \param nodePos the position of the destination node
 * \return Ipv4Address of a coordinator closer to the destination, Ipv4Address::GetZero () if no coordinator was found
 */
Ipv4Address 
PositionTable::GetCoordinatorFromNeighbor (Vector position, Vector nodePos)
{
  //no need to check if table is empty, was checked in BestNeighbor right before

  double initialDistance = CalculateDistance (nodePos, position);

  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i;
  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
      if (initialDistance > CalculateDistance (i->second.first, position) && GetIsCoordinator (i->first))
        {
          return i->first;
        }
    }

  return Ipv4Address::GetZero ();
}

/**
 * \brief Gets next hop according to GPCR protocol
 * \param position the position of the destination node
 * \param nodePos the position of the node that has the packet
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no neighbour was found in greedy mode
 */
Ipv4Address 
PositionTable::BestNeighbor (Vector position, Vector nodePos)
{
  Purge ();

  double initialDistance = CalculateDistance (nodePos, position);

  if (m_table.empty ())
    {
      NS_LOG_DEBUG ("BestNeighbor table is empty; Position: " << position);
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)

  Ipv4Address bestFoundID = GetCoordinatorFromNeighbor (position, nodePos);
  if(!(bestFoundID == Ipv4Address::GetZero ()))
    {
      NS_LOG_DEBUG("Found a Coordinator " << bestFoundID);
      return bestFoundID;
    }

  NS_LOG_DEBUG("No Coordinator found");

  bestFoundID = m_table.begin ()->first;
  double bestFoundDistance = CalculateDistance (m_table.begin ()->second.first, nodePos);
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i;
  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
      if (bestFoundDistance > CalculateDistance (i->second.first, nodePos))
        {
          bestFoundID = i->first;
          bestFoundDistance = CalculateDistance (i->second.first, nodePos);
        }
    }

  if(initialDistance > bestFoundDistance)
    return bestFoundID;
  else
    return Ipv4Address::GetZero (); //so it enters Recovery-mode

}


/**
 * \brief Gets next hop according to GPCR recovery-mode protocol (right hand rule)
 * \param previousHop the position of the node that sent the packet to this node
 * \param nodePos the position of the destination node
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address
PositionTable::BestAngle (Vector previousHop, Vector nodePos)
{
  Purge ();

  if (m_table.empty ())
    {
      NS_LOG_DEBUG ("BestNeighbor table is empty; Position: " << nodePos);
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)

  double tmpAngle;
  Ipv4Address bestFoundID = Ipv4Address::GetZero ();
  double bestFoundAngle = 360;
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > >::iterator i;

  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
      tmpAngle = GetAngle(nodePos, previousHop, i->second.first);
      if (bestFoundAngle > tmpAngle && tmpAngle != 0)
	{
	  bestFoundID = i->first;
	  bestFoundAngle = tmpAngle;
	}
    }
  if(bestFoundID == Ipv4Address::GetZero ()) //only if the only neighbour is who sent the packet
    {
      bestFoundID = m_table.begin ()->first;
    }
  return bestFoundID;
}


//Gives angle between the vector CentrePos-Refpos to the vector CentrePos-node counterclockwise
double 
PositionTable::GetAngle (Vector centrePos, Vector refPos, Vector node){
  double const PI = 4*atan(1);

  std::complex<double> A = std::complex<double>(centrePos.x,centrePos.y);
  std::complex<double> B = std::complex<double>(node.x,node.y);
  std::complex<double> C = std::complex<double>(refPos.x,refPos.y);   //Change B with C if you want angles clockwise

  std::complex<double> AB; //reference edge
  std::complex<double> AC;
  std::complex<double> tmp;
  std::complex<double> tmpCplx;

  std::complex<double> Angle;

  AB = B - A;
  AB = (real(AB)/norm(AB)) + (std::complex<double>(0.0,1.0)*(imag(AB)/norm(AB)));

  AC = C - A;
  AC = (real(AC)/norm(AC)) + (std::complex<double>(0.0,1.0)*(imag(AC)/norm(AC)));

  tmp = log(AC/AB);
  tmpCplx = std::complex<double>(0.0,-1.0);
  Angle = tmp*tmpCplx;
  Angle *= (180/PI);
  if (real(Angle)<0)
    Angle = 360+real(Angle);

  return real(Angle);

}





/**
 * \ProcessTxError
 */
void PositionTable::ProcessTxError (WifiMacHeader const & hdr)
{
}



}   // gpcr
} // ns3
