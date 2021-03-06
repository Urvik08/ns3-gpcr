#ifndef GPCR_PTABLE_H
#define GPCR_PTABLE_H

#include <map>
#include <cassert>
#include <stdint.h>
#include "ns3/ipv4.h"
#include "ns3/timer.h"
#include <sys/types.h>
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/random-variable.h"
#include <complex>

namespace ns3 {
namespace gpcr {

/*
 * \ingroup gpcr
 * \brief Position table used by GPCR
 */
class PositionTable
{
public:
  /// c-tor
  PositionTable ();

  /**
   * \brief Gets the info about if the node "id" is a coordinator
   * \param id Ipv4Address to get the isCoordinator from
   * \return uint8_t with 1 if it is a coordinator, 0 otherwise
   */
  uint8_t GetIsCoordinator (Ipv4Address id);


  /**
   * \brief Gets the info about if the node in, which called the function, is a coordinator
   * \return uint8_t with 1 if it is a coordinator, 0 otherwise
   */
  uint8_t AmICoordinator ();


  /**
   * \brief Gets the last time the entry was updated
   * \param id Ipv4Address to get time of update from
   * \return Time of last update to the position
   */
  Time GetEntryUpdateTime (Ipv4Address id);

  /**
   * \brief Adds entry in position table
   */
  void AddEntry (Ipv4Address id, Vector position, uint8_t isCoordinator);

  /**
   * \brief Deletes entry in position table
   */
  void DeleteEntry (Ipv4Address id);

  /**
   * \brief Gets position from position table
   * \param id Ipv4Address to get position from
   * \return Position of that id or NULL if not known
   */
  Vector GetPosition (Ipv4Address id);

  /**
   * \brief Checks if a node is a neighbour
   * \param id Ipv4Address of the node to check
   * \return True if the node is neighbour, false otherwise
   */
  bool isNeighbour (Ipv4Address id);

  /**
   * \brief remove entries with expired lifetime
   */
  void Purge ();

  /**
   * \brief clears all entries
   */
  void Clear ();

  /**
   * \Get Callback to ProcessTxError
   */
  Callback<void, WifiMacHeader const &> GetTxErrorCallback () const
  {
    return m_txErrorCallback;
  }

/**
 * \brief Gets a coordinator as next hop if thre is one
 * \param position the position of the node that has the packet
 * \param nodePos the position of the destination node
 * \return Ipv4Address of a coordinator closer to the destination, Ipv4Address::GetZero () if no coordinator was found
 */
  Ipv4Address GetCoordinatorFromNeighbor (Vector position, Vector nodePos);

  /**
   * \brief Gets next hop according to GPCR protocol
   * \param position the position of the destination node
   * \param nodePos the position of the node that has the packet
   * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
   */
  Ipv4Address BestNeighbor (Vector position, Vector nodePos);

  static Vector GetInvalidPosition ()
  {
    return Vector (-1, -1, 0);
  }

  /**
   * \brief Gets next hop according to GPCR recovery-mode protocol (right hand rule)
   * \param previousHop the position of the node that sent the packet to this node
   * \param nodePos the position of the destination node
   * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
   */
  Ipv4Address BestAngle (Vector previousHop, Vector nodePos);

  //Gives angle between the vector CentrePos-Refpos to the vector CentrePos-node counterclockwise
  double GetAngle (Vector centrePos, Vector refPos, Vector node);



private:
  Time m_entryLifeTime;
  std::map<Ipv4Address, std::pair<Vector, std::pair<Time, uint8_t> > > m_table;
  // TX error callback
  Callback<void, WifiMacHeader const &> m_txErrorCallback;
  // Process layer 2 TX error notification
  void ProcessTxError (WifiMacHeader const&);




};

}   // gpcr
} // ns3
#endif /* GPCR_PTABLE_H */
