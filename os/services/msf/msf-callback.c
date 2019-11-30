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
 *         MSF callback functions
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "assert.h"

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-num-cells.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/*---------------------------------------------------------------------------*/
void
msf_callback_joining_network(void)
{
  tsch_rpl_callback_joining_network();
  msf_activate();
}
/*---------------------------------------------------------------------------*/
void
msf_callback_leavning_network(void)
{
  msf_deactivate();
  tsch_rpl_callback_leaving_network();
}
/*---------------------------------------------------------------------------*/
int
msf_callback_packet_ready(void)
{
  /*
   * we're going to add an autonomous TX cell to the link-layer
   * destination if necessary.
   */
  const linkaddr_t *dest_addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  tsch_neighbor_t *nbr;

  if(msf_is_activated() == false || dest_addr == NULL) {
    /* nothing to do */
  } else {
    /*
     * packetbuf is expected to have a packet (frame) which is about to
     * be enqueued to the TX queue.
     */
    if(linkaddr_cmp(dest_addr, &linkaddr_null)) {
      /* we care only about unicast; do nothing. */
    } else if(msf_autonomous_cell_is_scheduled_tx(dest_addr) ||
              ((nbr = tsch_queue_get_nbr(dest_addr)) != NULL &&
               msf_negotiated_cell_is_scheduled_tx(nbr) == true)) {
      /* we've already had at least one TX cell; do nothing. */
    } else {
      /*
       * need to allocate an autonomous TX cell to the destination. If
       * we fail to add an autonomous cell, nbr->autonomoustx_cell will
       * have NULL.
       */
      msf_autonomous_cell_add_tx(dest_addr);
    }
  }

  /*
   * always return zero (non-negative value), which prevent the packet
   * from being dropped by the caller (tsch_queue_add_packet()).
   */
  return 0;
}
/*---------------------------------------------------------------------------*/
void
msf_callback_packet_sent(uint16_t slot_offset,
                         uint8_t mac_tx_status, int num_tx,
                         const linkaddr_t *dest_addr)
{
  /*
   * what we're going to do is basically either:
   *
   * - if MSF is not activated, do nothing
   * - elif we have negotiated cells to the neighbor, update the counters
   * - elif we don't have any packet in TX queue, delete the autonomous cell
   * - else keep the autonomous cell for a next unicast frame to the neighbor
   *
   */
  if(msf_is_activated() == false ||
     dest_addr == NULL ||
     linkaddr_cmp(dest_addr, &linkaddr_null)) {
    /* do nothing */
  } else {
    tsch_neighbor_t *nbr;
    const linkaddr_t *parent_addr;

    if((nbr = tsch_queue_get_nbr(dest_addr)) == NULL ||
       tsch_queue_is_empty(nbr)) {
      /* we don't need to keep the autonomous TX cell */
      msf_autonomous_cell_delete_tx(dest_addr);
    }

    if((parent_addr = msf_housekeeping_get_parent_addr()) !=  NULL &&
       linkaddr_cmp(dest_addr, parent_addr) &&
       msf_negotiated_cell_is_scheduled_tx(nbr)) {
      /* update the counters for the negotiated TX cells */
      msf_num_cells_update_tx_used(num_tx);
      msf_negotiated_cell_update_num_tx(slot_offset, num_tx, mac_tx_status);
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_callback_packet_recv(const struct tsch_asn_t *asn,
                         const linkaddr_t *src_addr)
{
  /*
   * The two slotframes for MSF have the same length; use one for the
   * autonomous cells in this callback
  */
  tsch_slotframe_t *slotframe = msf_autonomous_cell_get_slotframe();
  uint16_t slot_offset;
  if(slotframe == NULL || asn == NULL || src_addr == NULL) {
    /* nothing to do */
  } else if((slot_offset = TSCH_ASN_MOD(*asn, slotframe->size)) == 0) {
    /*
     *  ignore a reception on the shared cell in the minimum schedule
     *  (RFC8480); nothing to do
     */
  } else {
    const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
    if(parent_addr == NULL ||
       linkaddr_cmp(parent_addr, src_addr) == 0) {
      /* nothing to do */
    } else {
      msf_num_cells_increment_rx_used();
    }
    msf_negotiated_cell_rx_is_used(src_addr, slot_offset);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_callback_parent_switch(rpl_parent_t *old, rpl_parent_t *new)
{
  if(msf_is_activated() && tsch_is_coordinator == 0) {
    uip_ipaddr_t *ipaddr;
    const linkaddr_t *linkaddr;

    tsch_rpl_callback_parent_switch(old, new);

    if(new == NULL) {
      ipaddr = NULL;
    } else {
      ipaddr = rpl_parent_get_ipaddr(new);
    }

    if(ipaddr == NULL) {
      linkaddr = NULL;
    } else {
      linkaddr = (const linkaddr_t *)uip_ds6_nbr_lladdr_from_ipaddr(ipaddr);
    }

    msf_housekeeping_set_parent_addr(linkaddr);
  } else {
    /* do nothing */
  }
}
/*---------------------------------------------------------------------------*/
void
msf_callback_tsch_nbr_removed(tsch_neighbor_t *nbr)
{
  if(nbr == NULL) {
    /* do nothing */
  } else {
    /* delete an autnomous cell for the peer if it exists */
    msf_autonomous_cell_delete_tx(&nbr->addr);
  }
}
/*---------------------------------------------------------------------------*/
