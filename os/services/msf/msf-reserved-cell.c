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
 *         MSF Reserved Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "assert.h"

#include "lib/random.h"
#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"

#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* variables */
extern struct tsch_asn_divisor_t tsch_hopping_sequence_length;

/* static functions */
static int32_t find_unused_slot_offset(tsch_slotframe_t *slotframe);

/*---------------------------------------------------------------------------*/
static int32_t
find_unused_slot_offset(tsch_slotframe_t *slotframe)
{
  int32_t ret, slot_offset;
  uint16_t slot_offset_base;
  const tsch_link_t *autonomous_rx_cell = msf_autonomous_cell_get_rx();

  assert(autonomous_rx_cell != NULL);

  slot_offset_base = random_rand() % slotframe->size.val;
  ret = -1;
  for(int i = 0; i < slotframe->size.val; i++) {
    slot_offset = (slot_offset_base + i) % slotframe->size.val;
    if(tsch_schedule_get_link_by_timeslot(slotframe, slot_offset) == NULL &&
       slot_offset != 0 &&
       slot_offset != autonomous_rx_cell->timeslot) {
      /*
       * avoid using the slot offset of 0, which is used by the
       * minimal schedule, as well as the slot offset of the autonoous
       * RX cell
       */
      ret = slot_offset;
      break;
    } else {
      /* try the next one */
    }
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
int
msf_reserved_cell_get_num_cells(const linkaddr_t *peer_addr)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  tsch_link_t *cell;
  int ret;

  if(slotframe == NULL) {
    ret = 0;
  } else {
    ret = 0;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = (tsch_link_t *)list_item_next(cell)) {
      if((cell->link_options & LINK_OPTION_RESERVED_LINK) &&
         linkaddr_cmp(&cell->addr, peer_addr)) {
        /* this is a reserved cell for the peer */
        ret += 1;
      } else {
        /* this is not */
      }
    }
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_reserved_cell_get(const linkaddr_t *peer_addr,
                      int32_t slot_offset, int32_t channel_offset)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  tsch_link_t *cell;

  if(slotframe == NULL) {
    cell = NULL;
  } else {
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = (tsch_link_t *)list_item_next(cell)) {
      if((cell->link_options & LINK_OPTION_RESERVED_LINK) &&
         linkaddr_cmp(&cell->addr, peer_addr) &&
         (slot_offset < 0 || cell->timeslot == slot_offset) &&
         (channel_offset < 0 || cell->channel_offset == channel_offset)) {
        /* return the first found one */
        break;
      } else {
        /* try next */
      }
    }
  }

  return cell;
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_reserved_cell_add(const linkaddr_t *peer_addr,
                      msf_negotiated_cell_type_t cell_type,
                      int32_t slot_offset, int32_t channel_offset)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  tsch_link_t *cell;
  uint8_t link_options;
  int32_t _slot_offset;
  int32_t _channel_offset;

  assert(peer_addr != NULL);

  if(slotframe == NULL) {
    return NULL;
  }

  if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX) {
    link_options = LINK_OPTION_TX | LINK_OPTION_RESERVED_LINK;
  } else if(cell_type == MSF_NEGOTIATED_CELL_TYPE_RX) {
    link_options = LINK_OPTION_RX | LINK_OPTION_RESERVED_LINK;
  } else {
    /* unsupported */
    LOG_ERR("invalid negotiated cell type value: %u\n", cell_type);
    return NULL;
  }

  if(slot_offset < 0) {
    _slot_offset = find_unused_slot_offset(slotframe);
  } else if(tsch_schedule_get_link_by_timeslot(slotframe,
                                               slot_offset) != NULL) {
    /* this slot is used; we cannot reserve a cell */
    _slot_offset = -1;
  } else {
    _slot_offset = slot_offset;
  }

  if(_slot_offset < 0) {
    cell = NULL;
  } else {
    if(channel_offset < 0) {
      /* pick a channel offset */
      _channel_offset = random_rand() % tsch_hopping_sequence_length.val;
    } else if (channel_offset > (tsch_hopping_sequence_length.val - 1)) {
      /* invalid channel offset */
      _channel_offset = -1;
    } else {
      _channel_offset = channel_offset;
    }

    /* the reserved cell doesn't have any link option on */
    if(_channel_offset < 0 ||
       (cell = tsch_schedule_add_link(slotframe, link_options,
                                      LINK_TYPE_NORMAL, peer_addr,
                                      (uint16_t)_slot_offset,
                                      (uint16_t)_channel_offset)) == NULL) {
      cell = NULL;
      LOG_ERR("failed to reserve a cell at "
              "slot_offset:%ld, channel_offset:%ld\n",
              _slot_offset, _channel_offset);
    } else {
      LOG_DBG("reserved a cell at slot_offset:%u, channel_offset:%u\n",
              (uint16_t)_slot_offset, (uint16_t)_channel_offset);
    }
  }

  return cell;
}
/*---------------------------------------------------------------------------*/
void
msf_reserved_cell_delete_all(const linkaddr_t *peer_addr)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  tsch_link_t *cell;
  tsch_link_t *next_cell;

  assert(slotframe != NULL);

  for(cell = list_head(slotframe->links_list); cell != NULL; cell = next_cell) {
    next_cell = (tsch_link_t *)list_item_next(cell);
    if(cell->link_options & LINK_OPTION_RESERVED_LINK) {
      if(peer_addr == NULL ||
         linkaddr_cmp(&cell->addr, peer_addr)) {
        uint16_t slot_offset = cell->timeslot;
        uint16_t channel_offset = cell->channel_offset;
        msf_housekeeping_delete_cell_later(cell);
        LOG_DBG("released a reserved cell at "
                "slot_offset:%u, channel_offset:%u\n",
                slot_offset, channel_offset);
      }
    } else {
      /* this is not a reserved cell; skip it */
    }
  }
}
/*---------------------------------------------------------------------------*/
