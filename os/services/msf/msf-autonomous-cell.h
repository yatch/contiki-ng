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
 *         MSF Autonomous Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_AUTONOMOUS_CELL_H
#define _MSF_AUTONOMOUS_CELL_H

#include "net/mac/tsch/tsch.h"

/**
 * \brief Return a pointer to the slotframe for the autnomous cells
 * \return non-NULL if it exists, otherwise NULL
 */
tsch_slotframe_t *msf_autonomous_cell_get_slotframe(void);

/**
 * \brief Activate the autonomous cell scheduling
 * \details An autonomous RX cell will be installed.
 */
int msf_autonomous_cell_activate();

/**
 * \brief Deactivate the autonomous cell scheduling
 * \details All the autonomous cells scheduled will be deleted
 */
void msf_autonomous_cell_deactivate();

/**
 * \brief Return a poniter to the autonomous RX cell
 * \return non-NULL if it's scheduled, otherwise NULL
 */
const tsch_link_t *msf_autonomous_cell_get_rx(void);

/**
 * \brief Add an autonomous TX cell for a peer
 * \param peer_addr The MAC address of the target peer
 */
void msf_autonomous_cell_add_tx(const linkaddr_t *peer_addr);

/**
 * \brief Delete an autonomous TX cell scheduled for a peer
 * \param peer_addr The MAC address of the target peer
 */
void msf_autonomous_cell_delete_tx(const linkaddr_t *peer_addr);

/**
 * \brief Return whether an autonomous TX cell for a peer is scheduled
 * or not
 * \param peer_addr The MAC address of the target peer
 * \return true if it's scheduled, otherwise false
 */
bool msf_autonomous_cell_is_scheduled_tx(const linkaddr_t *peer_addr);

#endif /* !_MSF_AUTONOMOUS_CELL_H */
/** @} */
