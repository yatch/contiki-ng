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
 *         MSF Reserved Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_RESERVED_CELL_H_
#define _MSF_RESERVED_CELL_H_

#include <stdint.h>

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

/**
 * \brief Return the number of reserved cells for a peer
 * \param peer_addr The MAC address of the target peer
 */
int msf_reserved_cell_get_num_cells(const linkaddr_t *peer_addr);

/**
 * \breif Return a reserved cell matching conditions
 * \param peer_addr The MAC address of a target cell
 * \param slot_offset The slot offset of a target cell
 * \param channel_offset The channel offset of a target cell
 * \param non-NULL if found, otherwise NULL
 */
tsch_link_t *msf_reserved_cell_get(const linkaddr_t *peer_addr,
                                   int32_t slot_offset,
                                   int32_t channel_offset);

/**
 * \brief Add (reserve) a cell in the slotframe for negotiated cells
 * \param peer_addr The MAC address of the peer
 * \param cell_type Type of a reserved cell (TX or RX)
 * \param slot_offset The slot offset of a reserved cell
 * \param channel_offset The channel offset of a reserved cell
 */
tsch_link_t *msf_reserved_cell_add(const linkaddr_t *peer_addr,
                                   msf_negotiated_cell_type_t cell_type,
                                   int32_t slot_offset,
                                   int32_t channel_offset);

/**
 * \brief Delete all the cells reserved for a peer
 * \param peer_addr The MAC address of the peer
 */
void msf_reserved_cell_delete_all(const linkaddr_t *peer_addr);

#endif /* _MSF_RESERVED_CELL_H_ */
/** @} */
