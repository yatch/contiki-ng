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
 *         MSF callback functions
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_CALLBACK_H_
#define _MSF_CALLBACK_H_

#include <stdint.h>

#include "net/linkaddr.h"

/**
 * \brief Callback function on joining a network
 */
void msf_callback_joining_network(void);

/**
 * \brief Callback function on leaving a network
 */
void msf_callback_leavning_network(void);

/**
 * \brief Callback function on transmission of a frame
 * \param slot_offset The slot offset at which the last transmission occurs
 * \param mac_tx_status TX status of the transmission
 * \param num_tx The number of transmissions performed
 * \param dest_addr The destination address of the frame
 */
void msf_callback_packet_sent(uint16_t slot_offset,
                              uint8_t mac_tx_status, int num_tx,
                              const linkaddr_t *dest_addr);

/**
 * \brief Callback function on reception of a frame
 * \param asn ASN at which the reception occurs
 * \param src_addr The source address of the received frame
 */
void msf_callback_packet_recv(const struct tsch_asn_t *asn,
                              const linkaddr_t *src_addr);

/**
 * \brief Callback function on parent switch
 * \param old A pointer to the previous RPL preferred parent
 * \param new A pointer to the new RPL preferred parent
 */
void msf_callback_parent_switch(rpl_parent_t *old, rpl_parent_t * newone);

/**
 * \brief Callback function on removal of TSCH neighbor
 * \param nbr A pointer to a tsch_neighbor_t object to be removed
 */
void msf_callback_tsch_nbr_removed(tsch_neighbor_t *nbr);

#endif /* _!MSF_CALLBACK_H_ */
/** @} */
