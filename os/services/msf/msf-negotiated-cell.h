/*
 * Copyright (c) 2019, Inria.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup msf
 * @{
 */
/**
 * \file
 *         MSF Negotiated Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_NEGOTIATED_CELL_H
#define _MSF_NEGOTIATED_CELL_H

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

/* definitions */
/**
 * \brief Types of negotiated cells
 */
typedef enum {
  MSF_NEGOTIATED_CELL_TYPE_TX,  /**< Negotiated TX cell */
  MSF_NEGOTIATED_CELL_TYPE_RX   /**< Negotiated RX cell */
} msf_negotiated_cell_type_t;

/**
 * \brief Activate the negotiated cell scheduling
 */
void msf_negotiated_cell_activate(void);

/**
 * \brief Deactivate the negotiated cell scheduling
 * \details All the negotiated cells including reserved ones will be
 * deleted
 */
void msf_negotiated_cell_deactivate(void);

/**
 * \brief Return a pointer to the slotframe for negotiated cells
 */
tsch_slotframe_t *msf_negotiated_cell_get_slotframe(void);

/**
 * \brief Add a negotiated cell
 * \param peer_addr The MAC address of the peer
 * \param type Type of the negotiated cell (TX or RX)
 * \param slot_offset The slot offset of the cell
 * \param channel_offset The channel offset of the cell
 * \return 0 on success, -1 on failure
 */
int msf_negotiated_cell_add(const linkaddr_t *peer_addr,
                            msf_negotiated_cell_type_t type,
                            uint16_t slot_offset, uint16_t channel_offset);

/**
 * \brief Delete a negotiated cell
 * \param cell A pointer to the cell to delete
 */
void msf_negotiated_cell_delete(tsch_link_t *cell);

/**
 * \brief Delete all negotiated cells associated with a peer
 * \param peer_addr The MAC address of the target peer
 * \details Specify NULL for peer_addr to remove all the negotiated
 * cells in the schedule
 */
void msf_negotiated_cell_delete_all(const linkaddr_t *peer_addr);

/**
 * \brief Return whether a negotiated TX cell is scheduled with a peer
 * \param nbr A tsch_neighbor_t object for the peer
 * \return true if it is the case, otherwise false
 */
bool msf_negotiated_cell_is_scheduled_tx(tsch_neighbor_t *nbr);

/**
 * \brief Return a negotiated cell to delete
 * \param peer_addr The MAC address of the target peer
 * \param cell_type The type of a negotiated cell to delete
 * \return non-NULL if there is a candidate, otherwise NULL
 */
tsch_link_t *msf_negotiated_cell_get_cell_to_delete(
  const linkaddr_t *peer_addr,
  msf_negotiated_cell_type_t cell_type);

/**
 * \brief Return the number of negotiated cells scheduled with a peer
 * \param cell_type The type of negotiated cells of interest
 * \param peer_addr The MAC address of the target peer
 */
uint16_t msf_negotiated_cell_get_num_cells(msf_negotiated_cell_type_t cell_type,
                                           const linkaddr_t *peer_addr);

/**
 * \brief Update the NumTx counter
 * \param slot_offset The slot offset at which the last transmission occurrs
 * \param num_tx The number of transmissions performed for the frame
 * \param mac_tx_status The resuting TX status
 */
void msf_negotiated_cell_update_num_tx(uint16_t slot_offset,
                                       uint16_t num_tx, uint8_t mac_tx_status);

/**
 * \brief Get a cell to relocate
 * \return A pointer to a negotiated TX cell to relocate if any,
 * otherwise NULL
 */
tsch_link_t *msf_negotiated_cell_get_cell_to_relocate(void);

/**
 * \brief Return NumTx of a negotiated TX cell
 * \return The value of NumTX, or 0 if the given cell is not a negotiated
 * TX cell
 */
uint16_t msf_negotiated_cell_get_num_tx(tsch_link_t *cell);

/**
 * \brief Return NumTxAck of a negotiated TX cell
 * \return The value of NumTX, or 0 if the given cell is not a negotiated
 * TX cell
 */
uint16_t msf_negotiated_cell_get_num_tx_ack(tsch_link_t *cell);

/**
 * \brief Mark a negotiated RX cell as used
 * \param src_addr The src MAC address of a received packet
 * \param slot_offset The slot offste of the target negotiated RX cell
 */
void msf_negotiated_cell_rx_is_used(const linkaddr_t *src_addr,
                                    uint16_t slot_offset);

/**
 * \brief Detect and delete unused negotiated cells
 * \details Negotiated cells scheduled with a neighbor which is
 * thought being inactive will be delete
 */
void msf_negotiated_cell_delete_unused_cells(void);

#endif /* !_MSF_NEGOTIATED_CELL_H */
/** @} */
