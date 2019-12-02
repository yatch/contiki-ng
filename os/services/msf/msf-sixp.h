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
 *         MSF 6P-related common helpers (for internal use)
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_SIXP_COMMON_H_
#define _MSF_SIXP_COMMON_H_

#include <stdint.h>

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixp-pkt.h"

#include "msf-sixp-add.h"
#include "msf-sixp-clear.h"
#include "msf-sixp-count.h"
#include "msf-sixp-delete.h"
#include "msf-sixp-relocate.h"

/**
 * \brief Return a string for a specifired return code
 */
const char *msf_sixp_get_rc_str(sixp_pkt_rc_t rc);

/**
 * \brief Set cell parameters into a CellList buffer
 * \param buf Address in a CellList buffer to write cell parameters
 * \param cell The pointer to a target cell
 */
void msf_sixp_set_cell_params(uint8_t *buf, const tsch_link_t *cell);


/**
 * \brief Copy cell parameters from a CellList buffer
 * \param buf An address in a CellList buffer to read cell parameters
 * \param slot_offset A pointer to memory to copy the slot offset value
 * \param channel_offset A pointer to memory to copy the channel offset value
 */
void msf_sixp_get_cell_params(const uint8_t *buf,
                              uint16_t *slot_offset, uint16_t *channel_offset);

/**
 * \brief Return whether the waiting timer is expired or not
 * \return true if it's expired, otherwise false
 */
bool msf_sixp_is_request_wait_timer_expired(void);

/**
 * \brief Stop the waiting timer
 */
void msf_sixp_stop_request_wait_timer(void);

/**
 * \brief Start the waiting timer
 */
void msf_sixp_start_request_wait_timer(void);

/**
 * \brief Fill a CellList buffer with reserved cells
 * \param peer_addr MAC address of the target peer for which cells are reserved
 * \param cell_type Type of reserved cells (TX or RX)
 * \param cell_list A pointer to a CellList buffer
 * \param cell_list_len Length of the CellList buffer
 * \return the number of bytes written
 */
size_t msf_sixp_fill_cell_list(const linkaddr_t *peer_addr,
                               msf_negotiated_cell_type_t cell_type,
                               uint8_t *cell_list, size_t cell_list_len);

/**
 * \brief Reserve one of cells found in a given CellList
 * \param peer_addr MAC address of the peer
 * \param cell_type Type of a cell to reserve
 * \param cell_list A pointer to a CellList buffer
 * \param cell_list_len The length of the CellList buffer
 * \return A pointer to a reserved cell on success, otherwise NULL
 */
tsch_link_t *msf_sixp_reserve_one_cell(const linkaddr_t *peer_addr,
                                       msf_negotiated_cell_type_t cell_type,
                                       const uint8_t *cell_list,
                                       size_t cell_list_len);

/**
 * \brief Return a scheduled cell from a given CellList
 * \param peer_addr MAC address of the peer
 * \param link_options Link options of interest
 * \param cell_list A CellList buffer
 * \param cell_list_len Length of the CellList
 * \return non-NULL if found, otherwise NULL
 */
tsch_link_t *msf_sixp_find_scheduled_cell(const linkaddr_t *peer_addr,
                                          uint8_t link_options,
                                          const uint8_t *cell_list,
                                          size_t cell_list_len);
#endif /* !_MSF_SIXP_COMMON_H_ */
/** @} */
