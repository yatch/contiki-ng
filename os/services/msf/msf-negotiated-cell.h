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
 * \file
 *         MSF Negotiated Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef MSF_NEGOTIATED_CELL_H
#define MSF_NEGOTIATED_CELL_H

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

/* definitions */
typedef enum {
  MSF_NEGOTIATED_TX_CELL,
  MSF_NEGOTIATED_RX_CELL
} msf_negotiated_cell_type_t;

void msf_negotiated_cell_activate(void);
void msf_negotiated_cell_deactivate(void);
tsch_slotframe_t *msf_negotiated_cell_get_slotframe(void);

int msf_negotiated_cell_add(msf_negotiated_cell_type_t type,
                            const linkaddr_t *peer_addr,
                            uint16_t timeslot, uint16_t channel_offset);

void msf_negotiated_cell_delete(tsch_link_t *cell);
void msf_negotiated_cell_delete_all(const linkaddr_t *peer_addr);

bool msf_negotiated_cell_is_scheduled_tx(tsch_neighbor_t *nbr);
tsch_link_t *msf_negotiated_cell_get_tx_cell(tsch_neighbor_t *nbr);
unsigned int msf_negotiated_cell_get_num_tx_cells(const linkaddr_t *peer_addr);
void msf_negotiated_cell_update_num_tx(uint16_t slot_offset,
                                       uint16_t num_tx, uint8_t mac_tx_status);
tsch_link_t *msf_negotiated_cell_get_cell_to_relocate(void);
uint16_t msf_negotiated_cell_get_num_tx(tsch_link_t *cell);
uint16_t msf_negotiated_cell_get_num_tx_ack(tsch_link_t *cell);
#endif /* !MSF_NEGOTIATED_CELL_H */
